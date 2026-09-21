function G = iadpRiskGrid(map, cfg, ego, preds)
%IADPRISKGRID  Local dynamic cost map around the ego vehicle.
%
%   G = iadpRiskGrid(map, cfg, ego, preds)
%
%   G has the same geometric interface as the static map, so iadpW2G /
%   iadpG2W work on it directly.
%
%   G.cost     static drivability + soft shoulder + predicted-agent risk
%   G.blocked  logical, cost above cfg.grid.blockThr
%   G.static   static component only (for plotting)
%
%   Risk from an agent is a Gaussian whose width grows with prediction
%   uncertainty (class erraticness) and whose amplitude decays with time
%   to horizon, summed over every prediction mode. That produces exactly
%   the behaviour wanted on unstructured roads: a cow gets a big soft
%   blob, a bus gets a tight hard one.

res = cfg.grid.res;
x0  = ego.x - cfg.grid.behind;   x1 = ego.x + cfg.grid.ahead;
y0  = ego.y - cfg.grid.side;     y1 = ego.y + cfg.grid.side;

% keep the window square-ish regardless of heading
r   = max(cfg.grid.ahead, cfg.grid.side);
x0  = ego.x - r; x1 = ego.x + r;
y0  = ego.y - r; y1 = ego.y + r;

G.res  = res;
G.xmin = x0;  G.ymin = y0;
G.nx   = ceil((x1-x0)/res);
G.ny   = ceil((y1-y0)/res);

%% ---- static component -------------------------------------------------
[cc, rr] = meshgrid(1:G.nx, 1:G.ny);
[gx, gy] = iadpG2W(G, cc, rr);
[mc, mr, ok] = iadpW2G(map, gx, gy);
lin = sub2ind([map.ny map.nx], mr, mc);
stat = map.cost(lin);
stat(~ok) = 1e4;
G.static = stat;
G.cost   = stat;

%% ---- dynamic component ------------------------------------------------
if nargin >= 4 && ~isempty(preds)
    step = 2;                       % use every 2nd prediction step
    for i = 1:numel(preds)
        P = preds(i);
        for m = 1:numel(P.mode)
            w = P.mode(m).w;
            if w < 0.02, continue; end
            for k = 1:step:numel(P.t)
                c  = P.mode(m).xy(k,:);
                s  = sqrt(P.sig(k)^2 + (P.r + cfg.safe.minClear)^2);
                amp= w*cfg.grid.wDyn/(1 + 0.6*P.t(k));
                G  = stamp(G, c, s, amp);
            end
        end
        % the agent's current footprint is a hard obstacle
        G = stamp(G, P.pos0, max(P.r,0.5)*0.75, cfg.grid.blockThr*1.6);
    end
end

G.blocked = G.cost > cfg.grid.blockThr;
end

% =======================================================================
function G = stamp(G, c, s, amp)
%STAMP  Add amp*exp(-d^2/2s^2) around world point c (vectorised window).
rad = min(2.6*s, 8);
[c0, r0] = iadpW2G(G, c(1)-rad, c(2)-rad);
[c1, r1] = iadpW2G(G, c(1)+rad, c(2)+rad);
if c1 < c0 || r1 < r0, return; end
cols = c0:c1; rows = r0:r1;
if isempty(cols) || isempty(rows), return; end
[CC, RR] = meshgrid(cols, rows);
[X, Y]   = iadpG2W(G, CC, RR);
D2 = (X-c(1)).^2 + (Y-c(2)).^2;
G.cost(rows, cols) = G.cost(rows, cols) + amp*exp(-D2/(2*s^2));
end
