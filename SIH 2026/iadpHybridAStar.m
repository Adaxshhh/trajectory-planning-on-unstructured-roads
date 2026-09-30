function [path, info] = iadpHybridAStar(G, cfg, start, goal)
%IADPHYBRIDASTAR  Kinematically feasible search over the risk grid.
%
%   [path, info] = iadpHybridAStar(G, cfg, start, goal)
%
%   start, goal : [x y psi]   (rear-axle pose, world frame)
%   path        : [N x 3]     x, y, psi  (empty if no solution)
%   info        : .nodes .time .cost .reason
%
%   This is the *unstructured* planner: it needs no lanes, no centre line
%   and no road graph, only free space. It is what recovers the vehicle
%   when the lane-relative local planner has no feasible option, e.g. a
%   bus blocking the whole carriageway on a village road, forcing a
%   shoulder manoeuvre.
%
%   Implemented with an explicit binary heap (no toolbox dependency).

tic;
res   = cfg.hybrid.res;
nTh   = cfg.hybrid.nTheta;
dTh   = 2*pi/nTh;
nx    = ceil(G.nx*G.res/res);
ny    = ceil(G.ny*G.res/res);
maxN  = cfg.hybrid.maxNodes;

path = []; info.nodes = 0; info.cost = inf; info.reason = 'ok';

if ~inBounds(G, start(1), start(2)) || ~inBounds(G, goal(1), goal(2))
    info.reason = 'endpoint outside grid'; info.time = toc; return;
end

% ---- storage ----------------------------------------------------------
Nmax = maxN + 16;
nodeX = zeros(Nmax,1); nodeY = zeros(Nmax,1); nodeT = zeros(Nmax,1);
nodeG = inf(Nmax,1);   nodeP = zeros(Nmax,1); nodeS = zeros(Nmax,1);
closed = false(nx*ny*nTh, 1);          % dense closed set: O(1), no hashing

heapKey = zeros(Nmax,1); heapIdx = zeros(Nmax,1); hn = 0;

steers = cfg.hybrid.steers * cfg.veh.maxSteer;
count  = 1;
nodeX(1) = start(1); nodeY(1) = start(2); nodeT(1) = wrap(start(3));
nodeG(1) = 0; nodeP(1) = 0; nodeS(1) = 0;
[hn, heapKey, heapIdx] = push(hn, heapKey, heapIdx, h(start, goal), 1);

found = 0;
while hn > 0 && count < maxN
    [hn, heapKey, heapIdx, cur] = pop(hn, heapKey, heapIdx);
    kx = key(nodeX(cur), nodeY(cur), nodeT(cur), G, res, dTh, nx, ny);
    if closed(kx), continue; end
    closed(kx) = true;

    % goal test
    dxy = hypot(nodeX(cur)-goal(1), nodeY(cur)-goal(2));
    if dxy < cfg.hybrid.goalTolXY && abs(wrap(nodeT(cur)-goal(3))) < cfg.hybrid.goalTolH
        found = cur; break;
    end

    for si = 1:numel(steers)
        st = steers(si);
        [xn, yn, tn, ok, cRisk] = propagate(nodeX(cur), nodeY(cur), nodeT(cur), ...
                                            st, cfg, G);
        if ~ok, continue; end
        kx2 = key(xn, yn, tn, G, res, dTh, nx, ny);
        if closed(kx2), continue; end

        gNew = nodeG(cur) + cfg.hybrid.stepLen ...
             + cfg.hybrid.wSteer*abs(st)/cfg.veh.maxSteer*cfg.hybrid.stepLen ...
             + cfg.hybrid.wSwitch*abs(st - nodeS(cur))/cfg.veh.maxSteer ...
             + cfg.hybrid.wRisk*cRisk;

        count = count + 1;
        if count > maxN, break; end
        nodeX(count)=xn; nodeY(count)=yn; nodeT(count)=tn;
        nodeG(count)=gNew; nodeP(count)=cur; nodeS(count)=st;
        [hn, heapKey, heapIdx] = push(hn, heapKey, heapIdx, ...
                                      gNew + h([xn yn tn], goal), count);
    end
end

info.nodes = count;
info.time  = toc;

if found == 0
    info.reason = 'no solution within node budget';
    return;
end

% ---- reconstruct ------------------------------------------------------
idx = found; P = [];
while idx ~= 0
    P(end+1,:) = [nodeX(idx) nodeY(idx) nodeT(idx)]; %#ok<AGROW>
    idx = nodeP(idx);
end
path = flipud(P);
info.cost = nodeG(found);
end

% =======================================================================
function [x, y, th, ok, cRisk] = propagate(x, y, th, steer, cfg, G)
nSub  = 3;
ds    = cfg.hybrid.stepLen/nSub;
ok    = true; cRisk = 0;
for k = 1:nSub
    th = wrap(th + ds*tan(steer)/cfg.veh.L);
    x  = x + ds*cos(th);
    y  = y + ds*sin(th);
    [free, c] = checkPose(G, cfg, x, y, th);
    cRisk = cRisk + c;
    if ~free, ok = false; return; end
end
end

function [free, c] = checkPose(G, cfg, x, y, psi)
[ctr, rad] = iadpEgoDiscs(cfg, x, y, psi);
free = true; c = 0;
nr = max(1, round(rad/G.res));
for k = 1:size(ctr,1)
    [cc, rr, ok] = iadpW2G(G, ctr(k,1), ctr(k,2));
    if ~ok, free = false; return; end
    c0 = max(1, cc-nr); c1 = min(G.nx, cc+nr);
    r0 = max(1, rr-nr); r1 = min(G.ny, rr+nr);
    blk = G.blocked(r0:r1, c0:c1);
    if any(blk(:)), free = false; return; end
    c = c + G.cost(rr, cc);
end
end

function tf = inBounds(G, x, y)
[~,~,tf] = iadpW2G(G, x, y);
end

function v = h(a, b)
% Euclidean + heading-alignment penalty (consistent enough in practice)
v = hypot(a(1)-b(1), a(2)-b(2)) + 0.8*abs(wrap(a(3)-b(3)));
end

function k = key(x, y, th, G, res, dTh, nx, ny)
nTh = round(2*pi/dTh);
ix = min(max(floor((x - G.xmin)/res), 0), nx-1);
iy = min(max(floor((y - G.ymin)/res), 0), ny-1);
it = mod(floor((wrap(th)+pi)/dTh), nTh);
k  = (ix*ny + iy)*nTh + it + 1;          % 1-based linear index
end

function a = wrap(a)
a = mod(a + pi, 2*pi) - pi;
end

% ---------------- binary min-heap --------------------------------------
function [n, K, I] = push(n, K, I, k, i)
n = n + 1; K(n) = k; I(n) = i;
c = n;
while c > 1
    p = floor(c/2);
    if K(p) <= K(c), break; end
    [K(p),K(c)] = deal(K(c),K(p));
    [I(p),I(c)] = deal(I(c),I(p));
    c = p;
end
end

function [n, K, I, i] = pop(n, K, I)
i = I(1);
K(1) = K(n); I(1) = I(n); n = n - 1;
p = 1;
while true
    l = 2*p; r = l + 1; m = p;
    if l <= n && K(l) < K(m), m = l; end
    if r <= n && K(r) < K(m), m = r; end
    if m == p, break; end
    [K(p),K(m)] = deal(K(m),K(p));
    [I(p),I(m)] = deal(I(m),I(p));
    p = m;
end
end
