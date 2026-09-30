function [s, d, idx, hdg] = iadpPathProject(P, x, y, idxHint)
%IADPPATHPROJECT  Frenet projection of (x,y) onto reference path P.
%
%   [s,d,idx,hdg] = iadpPathProject(P, x, y)
%   [s,d,idx,hdg] = iadpPathProject(P, x, y, idxHint)  % local search window
%
%   s   arc length of the closest point
%   d   signed lateral offset (+ = left of path direction)
%   idx index of the closest sample
%   hdg path heading at that point

N = size(P.xy,1);
if nargin >= 4 && ~isempty(idxHint)
    lo = max(1, idxHint-80); hi = min(N, idxHint+160);
else
    lo = 1; hi = N;
end
seg = lo:hi;
dx  = P.xy(seg,1) - x;  dy = P.xy(seg,2) - y;
[~,j] = min(dx.^2 + dy.^2);
idx = seg(j);

% refine with the local tangent
hdg = P.hdg(idx);
t   = [cos(hdg) sin(hdg)];
n   = [-sin(hdg) cos(hdg)];
v   = [x - P.xy(idx,1), y - P.xy(idx,2)];
s   = P.s(idx) + v*t.';
d   = v*n.';
s   = min(max(s, P.s(1)), P.s(end));
end
