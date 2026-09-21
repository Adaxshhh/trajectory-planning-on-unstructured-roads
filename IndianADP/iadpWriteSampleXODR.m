function files = iadpWriteSampleXODR(cfg)
%IADPWRITESAMPLEXODR  Create OpenDRIVE scenes equivalent to the RoadRunner exports.
%
%   files = iadpWriteSampleXODR(cfg)
%
%   Writes two standards-compliant OpenDRIVE 1.6 files into cfg.io.xodrDir:
%     1) IndianVillageRoad.xodr        - narrow, curved, unmarked village road
%     2) IndianUrbanIntersection.xodr  - 4-way unsignalised urban intersection
%     3) IndianHighwayMerge.xodr       - highway + on-ramp for the merge case
%
%   These are the *same* files RoadRunner produces with
%   File > Export > OpenDRIVE, so iadpImportOpenDRIVE reads either source
%   with no change. If you already have RoadRunner .xodr exports, drop them
%   in cfg.io.xodrDir and they will be preferred.

if nargin < 1, cfg = iadpSetup; end
d = cfg.io.xodrDir;
files = {};

files{end+1} = writeVillage(fullfile(d,'IndianVillageRoad.xodr'));
files{end+1} = writeIntersection(fullfile(d,'IndianUrbanIntersection.xodr'));
files{end+1} = writeHighwayMerge(fullfile(d,'IndianHighwayMerge.xodr'));

if cfg.io.verbose
    fprintf('[iadp] wrote %d OpenDRIVE scenes to %s\n', numel(files), d);
end
end

% =======================================================================
function f = writeVillage(f)
% Unmarked village road: 2.6 m lanes, soft shoulders, S-curve.
g = {};
g{end+1} = geomLine(  0,   0,    0,   0.0000, 45);
g{end+1} = geomArc(  45,  45,    0,   0.0000, 55,  0.0180);
g{end+1} = geomArc( 100,  98.3, 25.9, 0.9900, 55, -0.0180);
g{end+1} = geomLine(155, 148.0, 51.6, 0.0000, 50);

s = header('IndianVillageRoad', -200, -60, 320, 160);
s = [s road('VillageRoad', 1, 205, g, laneSectionSym(2.60, 1.40, 'none', 'driving'))];
s = [s footer()];
writeFile(f, s);
end

% =======================================================================
function f = writeIntersection(f)
% 4-way unsignalised urban intersection, 2x2 lanes, 3.1 m wide.
A = 70;            % approach length
L = laneSectionSym2(3.10, 3.10, 0.80, 'broken');

g1 = { geomLine(0, -A-8,   0,        0,        A) };   % west  -> east
g2 = { geomLine(0,  A+8,   0,        pi,       A) };   % east  -> west
g3 = { geomLine(0,  0,    -A-8,      pi/2,     A) };   % south -> north
g4 = { geomLine(0,  0,     A+8,     -pi/2,     A) };   % north -> south
gC = { geomLine(0, -8,     0,        0,        16) };  % the box itself

s = header('IndianUrbanIntersection', -120, -120, 240, 240);
s = [s road('WestApproach',  1, A,  g1, L)];
s = [s road('EastApproach',  2, A,  g2, L)];
s = [s road('SouthApproach', 3, A,  g3, L)];
s = [s road('NorthApproach', 4, A,  g4, L)];
s = [s road('JunctionBox',   5, 16, gC, laneSectionSym2(3.10, 3.10, 0.0, 'none'))];
s = [s footer()];
writeFile(f, s);
end

% =======================================================================
function f = writeHighwayMerge(f)
% 2-lane highway with a 120 m acceleration ramp joining from the right.
gMain = { geomLine(0, -50, 0, 0, 320) };
gRamp = { geomLine(0, -30, -26, atan2(26,150), 152) };

s = header('IndianHighwayMerge', -100, -80, 420, 200);
s = [s road('Mainline', 1, 320, gMain, laneSectionSym2(3.50, 3.50, 2.50, 'broken'))];
s = [s road('OnRamp',   2, 152, gRamp, laneSectionSym(3.50, 1.50, 'broken', 'driving'))];
s = [s footer()];
writeFile(f, s);
end

% =======================================================================
% OpenDRIVE building blocks
% =======================================================================
function g = geomLine(s, x, y, hdg, len)
g = sprintf(['      <geometry s="%.6f" x="%.6f" y="%.6f" hdg="%.6f" length="%.6f">\n' ...
             '        <line/>\n      </geometry>\n'], s, x, y, hdg, len);
end

function g = geomArc(s, x, y, hdg, len, curv)
g = sprintf(['      <geometry s="%.6f" x="%.6f" y="%.6f" hdg="%.6f" length="%.6f">\n' ...
             '        <arc curvature="%.8f"/>\n      </geometry>\n'], s, x, y, hdg, len, curv);
end

function L = laneSectionSym(wDrive, wShoulder, mark, typ)
% one driving lane per direction + shoulder
L = sprintf([ ...
 '      <laneSection s="0.0">\n' ...
 '        <left>\n' ...
 '          <lane id="2" type="shoulder" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="1" type="%s" level="false">\n%s%s          </lane>\n' ...
 '        </left>\n' ...
 '        <center>\n          <lane id="0" type="none" level="false">\n%s          </lane>\n        </center>\n' ...
 '        <right>\n' ...
 '          <lane id="-1" type="%s" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="-2" type="shoulder" level="false">\n%s%s          </lane>\n' ...
 '        </right>\n' ...
 '      </laneSection>\n'], ...
 widthEl(wShoulder), markEl('none'), ...
 typ, widthEl(wDrive), markEl(mark), ...
 markEl(mark), ...
 typ, widthEl(wDrive), markEl(mark), ...
 widthEl(wShoulder), markEl('none'));
end

function L = laneSectionSym2(wIn, wOut, wShoulder, mark)
% two driving lanes per direction + shoulder
L = sprintf([ ...
 '      <laneSection s="0.0">\n' ...
 '        <left>\n' ...
 '          <lane id="3" type="shoulder" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="2" type="driving" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="1" type="driving" level="false">\n%s%s          </lane>\n' ...
 '        </left>\n' ...
 '        <center>\n          <lane id="0" type="none" level="false">\n%s          </lane>\n        </center>\n' ...
 '        <right>\n' ...
 '          <lane id="-1" type="driving" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="-2" type="driving" level="false">\n%s%s          </lane>\n' ...
 '          <lane id="-3" type="shoulder" level="false">\n%s%s          </lane>\n' ...
 '        </right>\n' ...
 '      </laneSection>\n'], ...
 widthEl(max(wShoulder,0.01)), markEl('none'), ...
 widthEl(wOut), markEl(mark), ...
 widthEl(wIn),  markEl(mark), ...
 markEl('solid'), ...
 widthEl(wIn),  markEl(mark), ...
 widthEl(wOut), markEl(mark), ...
 widthEl(max(wShoulder,0.01)), markEl('none'));
end

function w = widthEl(a)
w = sprintf('            <width sOffset="0.0" a="%.4f" b="0.0" c="0.0" d="0.0"/>\n', a);
end

function m = markEl(t)
m = sprintf('            <roadMark sOffset="0.0" type="%s" weight="standard" color="standard" width="0.12"/>\n', t);
end

function s = road(name, id, len, geoms, laneSec)
s = sprintf('  <road name="%s" length="%.6f" id="%d" junction="-1">\n', name, len, id);
s = [s sprintf('    <type s="0.0" type="town"><speed max="%.1f" unit="m/s"/></type>\n', 13.9)];
s = [s '    <planView>' newline];
for k = 1:numel(geoms), s = [s geoms{k}]; end %#ok<AGROW>
s = [s '    </planView>' newline];
s = [s '    <lanes>' newline '      <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>' newline];
s = [s laneSec];
s = [s '    </lanes>' newline '  </road>' newline];
end

function s = header(name, xmin, ymin, w, h)
s = sprintf(['<?xml version="1.0" encoding="UTF-8"?>\n<OpenDRIVE>\n' ...
    '  <header revMajor="1" revMinor="6" name="%s" version="1.00" date="%s" ' ...
    'north="%.4f" south="%.4f" east="%.4f" west="%.4f" vendor="IADP">\n' ...
    '    <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>\n' ...
    '  </header>\n'], name, datestr(now,'yyyy-mm-ddTHH:MM:SS'), ymin+h, ymin, xmin+w, xmin);
end

function s = footer()
s = sprintf('</OpenDRIVE>\n');
end

function writeFile(f, s)
fid = fopen(f,'w');
if fid < 0, error('iadp:io','Cannot write %s', f); end
fwrite(fid, s, 'char');
fclose(fid);
end
