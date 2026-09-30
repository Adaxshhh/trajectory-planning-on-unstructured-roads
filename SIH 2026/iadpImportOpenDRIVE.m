function odr = iadpImportOpenDRIVE(xodrFile, ds)
%IADPIMPORTOPENDRIVE  Parse an OpenDRIVE file (RoadRunner export or any other).
%
%   odr = iadpImportOpenDRIVE(file)       % 0.5 m sampling
%   odr = iadpImportOpenDRIVE(file, ds)
%
%   Supports every planView geometry defined by OpenDRIVE 1.4-1.7:
%       <line>  <arc>  <spiral>  <poly3>  <paramPoly3>
%   and lane widths given as cubic polynomials per <laneSection>.
%
%   Output
%     odr.name
%     odr.roads(i).id, .name, .length
%     odr.roads(i).center  [N x 3]  x, y, heading of the reference line
%     odr.roads(i).s       [N x 1]  arc length
%     odr.roads(i).leftEdge, .rightEdge   [N x 2] outer *driving* boundary
%     odr.roads(i).leftSh,   .rightSh     [N x 2] outer shoulder boundary
%     odr.roads(i).marked   logical  true if the road carries lane markings
%     odr.bbox  [xmin ymin xmax ymax]
%
%   The importer deliberately keeps both the driving boundary and the
%   shoulder boundary: on Indian roads the shoulder is often usable but
%   undesirable, which the risk grid encodes as a soft cost, not a wall.

if nargin < 2, ds = 0.5; end
if ~exist(xodrFile,'file'), error('iadp:xodr','File not found: %s', xodrFile); end

doc  = xmlread(xodrFile);
root = doc.getDocumentElement();

odr.file = xodrFile;
odr.name = charAttr(firstChild(root,'header'), 'name', 'unnamed');

roadNodes = root.getElementsByTagName('road');
roads = struct('id',{},'name',{},'length',{},'center',{},'s',{}, ...
               'leftEdge',{},'rightEdge',{},'leftSh',{},'rightSh',{},'marked',{});

for i = 0:roadNodes.getLength()-1
    rn = roadNodes.item(i);
    % direct children of <OpenDRIVE> only (skip nested)
    if ~strcmp(char(rn.getParentNode().getNodeName()), 'OpenDRIVE'), continue; end

    R.id     = str2double(charAttr(rn,'id','0'));
    R.name   = charAttr(rn,'name',sprintf('road%d',R.id));
    R.length = str2double(charAttr(rn,'length','0'));

    geoms = parseGeometries(rn);
    if isempty(geoms), continue; end
    if ~(R.length > 0)
        R.length = geoms(end).s + geoms(end).length;
    end

    sVec = (0:ds:R.length)';
    if sVec(end) < R.length - 1e-6, sVec(end+1) = R.length; end %#ok<AGROW>

    C = zeros(numel(sVec),3);
    for k = 1:numel(sVec)
        [C(k,1), C(k,2), C(k,3)] = evalReference(geoms, sVec(k));
    end

    [lanes, marked] = parseLanes(rn);
    [le, re, ls, rs] = laneBoundaries(C, sVec, lanes);

    R.center = C;  R.s = sVec;
    R.leftEdge = le;  R.rightEdge = re;
    R.leftSh   = ls;  R.rightSh   = rs;
    R.marked   = marked;
    roads(end+1) = R; %#ok<AGROW>
end

if isempty(roads)
    error('iadp:xodr','No <road> elements parsed from %s', xodrFile);
end
odr.roads = roads;

allPts = vertcat(roads.leftSh, roads.rightSh);
odr.bbox = [min(allPts(:,1)) min(allPts(:,2)) max(allPts(:,1)) max(allPts(:,2))];
end

% =======================================================================
function g = parseGeometries(roadNode)
g = struct('type',{},'s',{},'x',{},'y',{},'hdg',{},'length',{},'p',{});
pv = firstChild(roadNode,'planView');
if isempty(pv), return; end
gn = pv.getElementsByTagName('geometry');
for k = 0:gn.getLength()-1
    n = gn.item(k);
    G.s   = str2double(charAttr(n,'s','0'));
    G.x   = str2double(charAttr(n,'x','0'));
    G.y   = str2double(charAttr(n,'y','0'));
    G.hdg = str2double(charAttr(n,'hdg','0'));
    G.length = str2double(charAttr(n,'length','0'));
    G.p = struct();
    if ~isempty(firstChild(n,'line'))
        G.type = 'line';
    elseif ~isempty(firstChild(n,'arc'))
        G.type = 'arc';
        G.p.curv = str2double(charAttr(firstChild(n,'arc'),'curvature','0'));
    elseif ~isempty(firstChild(n,'spiral'))
        G.type = 'spiral';
        sp = firstChild(n,'spiral');
        G.p.cStart = str2double(charAttr(sp,'curvStart','0'));
        G.p.cEnd   = str2double(charAttr(sp,'curvEnd','0'));
    elseif ~isempty(firstChild(n,'poly3'))
        G.type = 'poly3';
        p = firstChild(n,'poly3');
        G.p.a = str2double(charAttr(p,'a','0'));
        G.p.b = str2double(charAttr(p,'b','0'));
        G.p.c = str2double(charAttr(p,'c','0'));
        G.p.d = str2double(charAttr(p,'d','0'));
    elseif ~isempty(firstChild(n,'paramPoly3'))
        G.type = 'paramPoly3';
        p = firstChild(n,'paramPoly3');
        for f = {'aU','bU','cU','dU','aV','bV','cV','dV'}
            G.p.(f{1}) = str2double(charAttr(p,f{1},'0'));
        end
        G.p.pRange = charAttr(p,'pRange','normalized');
    else
        G.type = 'line';
    end
    g(end+1) = G; %#ok<AGROW>
end
[~,ix] = sort([g.s]); g = g(ix);
end

% =======================================================================
function [x,y,h] = evalReference(g, s)
% locate geometry segment
k = find([g.s] <= s + 1e-9, 1, 'last');
if isempty(k), k = 1; end
G  = g(k);
du = max(0, min(s - G.s, G.length));

switch G.type
    case 'line'
        x = G.x + du*cos(G.hdg);
        y = G.y + du*sin(G.hdg);
        h = G.hdg;
    case 'arc'
        c = G.p.curv;
        if abs(c) < 1e-10
            x = G.x + du*cos(G.hdg); y = G.y + du*sin(G.hdg); h = G.hdg;
        else
            R  = 1/c;  dh = c*du;  h = G.hdg + dh;
            x = G.x + R*( sin(G.hdg+dh) - sin(G.hdg));
            y = G.y + R*(-cos(G.hdg+dh) + cos(G.hdg));
        end
    case 'spiral'
        [x,y,h] = spiralPoint(G, du);
    case 'poly3'
        u = du;
        v = G.p.a + G.p.b*u + G.p.c*u^2 + G.p.d*u^3;
        dv= G.p.b + 2*G.p.c*u + 3*G.p.d*u^2;
        [x,y] = localToGlobal(G, u, v);
        h = G.hdg + atan2(dv,1);
    case 'paramPoly3'
        if strcmpi(G.p.pRange,'arcLength'), p = du; else, p = du/max(G.length,eps); end
        u  = G.p.aU + G.p.bU*p + G.p.cU*p^2 + G.p.dU*p^3;
        v  = G.p.aV + G.p.bV*p + G.p.cV*p^2 + G.p.dV*p^3;
        du_= G.p.bU + 2*G.p.cU*p + 3*G.p.dU*p^2;
        dv_= G.p.bV + 2*G.p.cV*p + 3*G.p.dV*p^2;
        [x,y] = localToGlobal(G, u, v);
        h = G.hdg + atan2(dv_, du_);
    otherwise
        x = G.x; y = G.y; h = G.hdg;
end
end

function [x,y] = localToGlobal(G, u, v)
x = G.x + u*cos(G.hdg) - v*sin(G.hdg);
y = G.y + u*sin(G.hdg) + v*cos(G.hdg);
end

function [x,y,h] = spiralPoint(G, du)
% Clothoid: curvature varies linearly. Numerically integrated (robust,
% no Fresnel special functions needed).
n  = max(8, ceil(du/0.1));
t  = linspace(0, du, n+1);
dc = (G.p.cEnd - G.p.cStart)/max(G.length,eps);
hh = G.hdg + G.p.cStart*t + 0.5*dc*t.^2;
xs = cumtrapz(t, cos(hh));
ys = cumtrapz(t, sin(hh));
x  = G.x + xs(end);
y  = G.y + ys(end);
h  = hh(end);
end

% =======================================================================
function [lanes, marked] = parseLanes(roadNode)
% Returns width polynomials of every lane of the FIRST lane section
% (sufficient for the scenes used here; multi-section roads fall back to
% the section covering s=0 which RoadRunner exports for constant profiles).
lanes.left = []; lanes.right = []; marked = false;
ln = firstChild(roadNode,'lanes');
if isempty(ln), return; end
secs = ln.getElementsByTagName('laneSection');
if secs.getLength()==0, return; end
sec = secs.item(0);

lanes.left  = sideLanes(sec,'left');
lanes.right = sideLanes(sec,'right');

rm = sec.getElementsByTagName('roadMark');
for k = 0:rm.getLength()-1
    t = charAttr(rm.item(k),'type','none');
    if ~strcmpi(t,'none'), marked = true; break; end
end
end

function L = sideLanes(sec, side)
L = struct('id',{},'type',{},'w',{});
sn = firstChild(sec, side);
if isempty(sn), return; end
ln = sn.getElementsByTagName('lane');
for k = 0:ln.getLength()-1
    n = ln.item(k);
    S.id   = str2double(charAttr(n,'id','0'));
    S.type = charAttr(n,'type','driving');
    wn = n.getElementsByTagName('width');
    if wn.getLength() > 0
        w = wn.item(0);
        S.w = [str2double(charAttr(w,'a','0')) str2double(charAttr(w,'b','0')) ...
               str2double(charAttr(w,'c','0')) str2double(charAttr(w,'d','0'))];
    else
        S.w = [3.5 0 0 0];
    end
    L(end+1) = S; %#ok<AGROW>
end
% order inner -> outer
if ~isempty(L)
    [~,ix] = sort(abs([L.id])); L = L(ix);
end
end

% =======================================================================
function [le, re, ls, rs] = laneBoundaries(C, sVec, lanes)
n = size(C,1);
le = zeros(n,2); re = zeros(n,2); ls = zeros(n,2); rs = zeros(n,2);
for k = 1:n
    nrm = [-sin(C(k,3)) cos(C(k,3))];    % +t direction (left)
    tDrvL = 0; tAllL = 0;
    for j = 1:numel(lanes.left)
        w = polyw(lanes.left(j).w, sVec(k));
        tAllL = tAllL + w;
        if isDriv(lanes.left(j).type), tDrvL = tAllL; end
    end
    tDrvR = 0; tAllR = 0;
    for j = 1:numel(lanes.right)
        w = polyw(lanes.right(j).w, sVec(k));
        tAllR = tAllR + w;
        if isDriv(lanes.right(j).type), tDrvR = tAllR; end
    end
    if tAllL == 0, tAllL = 3.5; tDrvL = 3.5; end
    if tAllR == 0, tAllR = 3.5; tDrvR = 3.5; end
    le(k,:) = C(k,1:2) + nrm*tDrvL;
    ls(k,:) = C(k,1:2) + nrm*tAllL;
    re(k,:) = C(k,1:2) - nrm*tDrvR;
    rs(k,:) = C(k,1:2) - nrm*tAllR;
end
end

function b = isDriv(t)
b = any(strcmpi(t,{'driving','restricted','parking','bidirectional'}));
end

function w = polyw(c, ds)
w = c(1) + c(2)*ds + c(3)*ds^2 + c(4)*ds^3;
w = max(w,0);
end

% =======================================================================
function n = firstChild(node, name)
n = [];
if isempty(node), return; end
ch = node.getChildNodes();
for k = 0:ch.getLength()-1
    if strcmp(char(ch.item(k).getNodeName()), name)
        n = ch.item(k); return;
    end
end
end

function v = charAttr(node, name, def)
v = def;
if isempty(node), return; end
if node.hasAttribute(name)
    v = char(node.getAttribute(name));
end
end
