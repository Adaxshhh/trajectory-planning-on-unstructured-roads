function map = iadpBuildMap(odr, cfg, routeIdx, startXY)
%IADPBUILDMAP  Static drivable map + route reference path from OpenDRIVE.
%
%   map = iadpBuildMap(odr, cfg)
%   map = iadpBuildMap(odr, cfg, routeIdx, startXY)
%
%   routeIdx : vector of road indices the ego must traverse, in order
%   startXY  : [x y] near which the route should begin (picks direction)
%
%   map.drivable  logical  [ny x nx]  inside a *driving* lane
%   map.soft      logical             shoulder / unclear edge band
%   map.cost      double              static traversal cost (0 = free)
%   map.route     reference path struct (see iadpMakePath)
%
%   Indian-road specific: the shoulder is NOT a wall. It is a soft-cost
%   region so the planner can use it to squeeze past a pushcart or a
%   stopped bus, which is exactly what human drivers do here.

if nargin < 3 || isempty(routeIdx), routeIdx = 1:numel(odr.roads); end
if nargin < 4, startXY = []; end

res = cfg.grid.res;
pad = 8;
bb  = odr.bbox + [-pad -pad pad pad];

map.res  = res;
map.xmin = bb(1); map.ymin = bb(2);
map.nx   = ceil((bb(3)-bb(1))/res);
map.ny   = ceil((bb(4)-bb(2))/res);
map.bbox = bb;

map.drivable = false(map.ny, map.nx);
map.soft     = false(map.ny, map.nx);

[cc, rr] = meshgrid(1:map.nx, 1:map.ny);
[gx, gy] = iadpG2W(map, cc, rr);

for i = 1:numel(odr.roads)
    R = odr.roads(i);
    polyD = [R.leftEdge; flipud(R.rightEdge)];
    polyS = [R.leftSh;   flipud(R.rightSh)];
    % restrict the test to the road bounding box for speed
    bx = [min(polyS(:,1))-1 max(polyS(:,1))+1];
    by = [min(polyS(:,2))-1 max(polyS(:,2))+1];
    sel = gx >= bx(1) & gx <= bx(2) & gy >= by(1) & gy <= by(2);
    if ~any(sel(:)), continue; end
    inD = false(size(sel)); inS = false(size(sel));
    inD(sel) = inpolygon(gx(sel), gy(sel), polyD(:,1), polyD(:,2));
    inS(sel) = inpolygon(gx(sel), gy(sel), polyS(:,1), polyS(:,2));
    map.drivable = map.drivable | inD;
    map.soft     = map.soft     | (inS & ~inD);
end

% extra "unclear edge" band: on unmarked roads the physical edge is fuzzy,
% so dilate the shoulder outward by cfg.grid.edgeSoft metres.
nDil = max(1, round(cfg.grid.edgeSoft/res));
band = dilateMask(map.drivable | map.soft, nDil) & ~map.drivable;
map.soft = map.soft | band;

map.cost = zeros(map.ny, map.nx);
map.cost(map.soft)  = cfg.grid.wEdge;
map.cost(~(map.drivable | map.soft)) = 1e4;      % off-road / buildings

map.roads = odr.roads;
map.name  = odr.name;

%% ---- route --------------------------------------------------------
xy = chainRoute(odr, routeIdx, startXY, cfg);
map.route = iadpMakePath(xy, 0.5);
map.routeIdx = routeIdx;
end

% =======================================================================
function xy = chainRoute(odr, idxList, startXY, cfg)
% Concatenate road centrelines, flipping each so it continues the previous
% one, then bias to the left-hand driving side (India drives on the left).
xy = [];
prevEnd = startXY;
for k = 1:numel(idxList)
    R = odr.roads(idxList(k));
    C = R.center;
    fwd = true;
    if ~isempty(prevEnd)
        dStart = hypot(C(1,1)-prevEnd(1),   C(1,2)-prevEnd(2));
        dEnd   = hypot(C(end,1)-prevEnd(1), C(end,2)-prevEnd(2));
        fwd = dStart <= dEnd;
    end
    if fwd
        pts = C(:,1:2); hdg = C(:,3);
        wL  = laneHalfWidth(R, true);
    else
        pts = flipud(C(:,1:2)); hdg = flipud(C(:,3)) + pi;
        wL  = laneHalfWidth(R, false);
    end
    % lateral bias onto the left driving lane
    nrm = [-sin(hdg) cos(hdg)];
    pts = pts + nrm.*wL;
    if ~isempty(xy) && hypot(pts(1,1)-xy(end,1), pts(1,2)-xy(end,2)) < 1e-3
        pts(1,:) = [];
    end
    xy = [xy; pts]; %#ok<AGROW>
    prevEnd = xy(end,:);
end
if isempty(xy), error('iadp:route','Empty route'); end
end

function w = laneHalfWidth(R, fwd)
% half width of the near-side driving lane, i.e. where the ego should sit
wl = mean(hypot(R.leftEdge(:,1)-R.center(:,1),  R.leftEdge(:,2)-R.center(:,2)));
wr = mean(hypot(R.rightEdge(:,1)-R.center(:,1), R.rightEdge(:,2)-R.center(:,2)));
if fwd, w = wl/2; else, w = wr/2; end
if ~isfinite(w) || w <= 0, w = 1.6; end
end

% =======================================================================
function B = dilateMask(A, n)
% Square-kernel binary dilation using conv2 (no Image Processing Toolbox).
k = ones(2*n+1);
B = conv2(double(A), k, 'same') > 0;
end
