function preds = iadpPredict(tracks, cfg, map, ego)
%IADPPREDICT  Short-horizon multimodal prediction of surrounding agents.
%
%   preds = iadpPredict(tracks, cfg, map, ego)
%
%   Four hypotheses are propagated for every track, weighted by class:
%     1 CTRV       continue the estimated turn (default for vehicles)
%     2 CV         straight-line continuation
%     3 BRAKE      decelerating to a stop (carts, rickshaws stop anywhere)
%     4 INTENT     leave the carriageway / cross it perpendicular
%                  (pedestrians, cattle) or cut across toward the ego path
%                  (rickshaws, two-wheelers)
%
%   The key departure from lane-based prediction is that hypothesis 4 does
%   NOT use lane geometry. It uses the local free-space direction and the
%   agent's own heading, which is what actually governs behaviour where
%   there are no lanes.
%
%   preds(i).id .cls .r .pos0
%   preds(i).t          [1 x K] prediction times
%   preds(i).mode(m).w  weight (sums to 1)
%   preds(i).mode(m).xy [K x 2]
%   preds(i).sig        [1 x K] positional 1-sigma (shared across modes)

K  = round(cfg.pred.horizon/cfg.pred.dt);
tt = (1:K)*cfg.pred.dt;
preds = struct('id',{},'cls',{},'r',{},'pos0',{},'t',{},'sig',{},'mode',{});

for i = 1:numel(tracks)
    T  = tracks(i);
    p  = iadpClassParams(T.cls);
    x0 = T.x(1); y0 = T.x(2); v0 = T.x(3); psi0 = T.x(4); w0 = T.x(5);

    P.id = T.id; P.cls = T.cls; P.r = p.r; P.pos0 = [x0 y0];
    P.t  = tt;
    P.sig = cfg.pred.sig0 + cfg.pred.growth*p.erratic*tt + ...
            0.5*sqrt(max(T.P(1,1),0) + max(T.P(2,2),0));

    % ---- weights -------------------------------------------------------
    if p.vru
        w = [0.25 0.20 0.20 0.35];
    elseif p.laneBound
        w = [0.45 0.35 0.15 0.05];
    else                            % rickshaws, carts: anything goes
        w = [0.35 0.22 0.20 0.23];
    end
    if v0 < 0.4, w = w + [0 0 0.25 0.15]; end     % standing agent
    w = w/sum(w);

    % ---- mode 1: CTRV --------------------------------------------------
    M(1).w  = w(1);
    M(1).xy = propCTRV(x0,y0,v0,psi0,w0, tt);
    % ---- mode 2: CV ----------------------------------------------------
    M(2).w  = w(2);
    M(2).xy = propCTRV(x0,y0,v0,psi0,0, tt);
    % ---- mode 3: braking ----------------------------------------------
    aBr = -min(p.amax, v0/max(cfg.pred.horizon*0.6, 0.5));
    M(3).w  = w(3);
    M(3).xy = propAccel(x0,y0,v0,psi0,w0*0.5,aBr, tt);
    % ---- mode 4: intent ------------------------------------------------
    [psiI, vI] = intentDirection(T, p, map, ego);
    M(4).w  = w(4);
    M(4).xy = propCTRV(x0,y0,vI,psiI,0, tt);

    P.mode = M;
    preds(end+1) = P; %#ok<AGROW>
    clear M
end
end

% =======================================================================
function xy = propCTRV(x,y,v,psi,w, tt)
xy = zeros(numel(tt),2);
for k = 1:numel(tt)
    t = tt(k);
    if abs(w) > 1e-4
        xy(k,1) = x + v/w*( sin(psi+w*t) - sin(psi));
        xy(k,2) = y + v/w*(-cos(psi+w*t) + cos(psi));
    else
        xy(k,1) = x + v*cos(psi)*t;
        xy(k,2) = y + v*sin(psi)*t;
    end
end
end

function xy = propAccel(x,y,v,psi,w,a, tt)
xy = zeros(numel(tt),2);
for k = 1:numel(tt)
    t  = tt(k);
    tStop = (a < 0) * (-v/a) + (a >= 0)*inf;
    te = min(t, tStop);
    d  = v*te + 0.5*a*te^2;
    h  = psi + w*te;
    xy(k,1) = x + d*cos(h);
    xy(k,2) = y + d*sin(h);
end
end

% =======================================================================
function [psiI, vI] = intentDirection(T, p, map, ego)
%INTENTDIRECTION  Where would this agent go if it did something surprising?
x0 = T.x(1); y0 = T.x(2); v0 = T.x(3); psi0 = T.x(4);

if p.vru
    % Cross the carriageway: aim perpendicular to the local road direction,
    % on the side the agent is already facing.
    hRoad = localRoadHeading(map, x0, y0, psi0);
    side  = sign(wrapAngle(psi0 - hRoad));
    if side == 0, side = 1; end
    psiI  = hRoad + side*pi/2;
    vI    = max(v0, 0.8*p.vmax*0.5);
else
    % Vehicles: cut across toward the ego's path (the unsignalled merge /
    % squeeze that dominates Indian traffic).
    psiTo = atan2(ego.y - y0, ego.x - x0);
    dpsi  = wrapAngle(psiTo - psi0);
    psiI  = psi0 + max(min(dpsi, 0.6), -0.6);
    vI    = max(v0, 1.0);
end
psiI = wrapAngle(psiI);
end

function h = localRoadHeading(map, x, y, fallback)
h = fallback;
if isempty(map) || ~isfield(map,'route'), return; end
[~, ~, idx] = iadpPathProject(map.route, x, y);
h = map.route.hdg(idx);
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end
