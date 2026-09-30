function P = iadpMakePath(xy, ds)
%IADPMAKEPATH  Build a resampled reference path with s / heading / curvature.
%
%   P = iadpMakePath(xy)      % xy = [N x 2], ds = 0.5 m
%   P = iadpMakePath(xy, ds)
%
%   P.xy    [M x 2]   resampled points
%   P.s     [M x 1]   cumulative arc length
%   P.hdg   [M x 1]   tangent heading (rad, unwrapped)
%   P.curv  [M x 1]   signed curvature [1/m]
%   P.len   scalar    total length

if nargin < 2, ds = 0.5; end

% drop duplicates
keep = [true; sum(abs(diff(xy)),2) > 1e-6];
xy   = xy(keep,:);
if size(xy,1) < 2
    P = struct('xy',xy,'s',0,'hdg',0,'curv',0,'len',0); return;
end

d  = [0; cumsum(hypot(diff(xy(:,1)), diff(xy(:,2))))];
L  = d(end);
sq = (0:ds:L)';
if sq(end) < L - 1e-9, sq(end+1) = L; end

X = interp1(d, xy(:,1), sq, 'pchip');
Y = interp1(d, xy(:,2), sq, 'pchip');

% mild smoothing to remove sampling chatter (moving average, base MATLAB)
if numel(X) > 7
    X = smooth3p(X); Y = smooth3p(Y);
end

dx = gradient(X, ds); dy = gradient(Y, ds);
h  = unwrap(atan2(dy, dx));
ddx= gradient(dx, ds); ddy = gradient(dy, ds);
den= (dx.^2 + dy.^2).^1.5;
k  = (dx.*ddy - dy.*ddx) ./ max(den, 1e-9);

P.xy   = [X Y];
P.s    = sq;
P.hdg  = h;
P.curv = k;
P.len  = sq(end);
end

function y = smooth3p(x)
n = numel(x);
y = x;
y(2:n-1) = 0.25*x(1:n-2) + 0.5*x(2:n-1) + 0.25*x(3:n);
end
