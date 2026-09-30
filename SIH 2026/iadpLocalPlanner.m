function [traj, info] = iadpLocalPlanner(P, ego, preds, G, cfg, cmd)
%IADPLOCALPLANNER  Lateral-offset trajectory sampling around a reference.
%
%   [traj, info] = iadpLocalPlanner(P, ego, preds, G, cfg, cmd)
%
%   P     reference path (iadpMakePath) - the *route*, not a lane
%   preds output of iadpPredict
%   G     risk grid
%   cmd   .vTarget .latBias .allowShoulder .mode  (from the behaviour FSM)
%
%   traj  .t .x .y .psi .v .k .s .d   (empty if nothing is feasible)
%   info  .nCand .nFeasible .cost .fail .reason .dSel .collisionAt
%
%   The candidate set is continuous lateral offsets, NOT discrete lanes,
%   which is the only thing that works when lane markings are absent. The
%   reference is a route corridor centre; offsets up to +/- dMax are free
%   to use, with soft cost added by the risk grid wherever the surface is
%   shoulder rather than carriageway.

ds = 0.5;
[s0, d0, i0] = iadpPathProject(P, ego.x, ego.y);
sEnd = min(s0 + cfg.local.horizonS, P.len);
info = struct('nCand',0,'nFeasible',0,'cost',inf,'fail',true, ...
              'reason','','dSel',NaN,'collisionAt',NaN);
traj = [];

if sEnd - s0 < 2.0
    info.reason = 'goal reached / path exhausted';
    % still return a short stopping trajectory so the vehicle behaves
    traj = brakeTrajectory(ego, cfg);
    info.fail = false; info.dSel = d0; info.cost = 0;
    return;
end

ss  = (s0:ds:sEnd).';
n   = numel(ss);
Rxy = [interp1(P.s, P.xy(:,1), ss, 'linear'), interp1(P.s, P.xy(:,2), ss, 'linear')];
Rh  = interp1(P.s, unwrap(P.hdg), ss, 'linear');
Nrm = [-sin(Rh) cos(Rh)];

kappaMax = tan(cfg.veh.maxSteer)/cfg.veh.L * 0.9;

dTargets = linspace(-cfg.local.dMax, cfg.local.dMax, cfg.local.nLat) + cmd.latBias;
best = []; bestCost = inf;

for q = 1:numel(dTargets)
    dT = dTargets(q);
    info.nCand = info.nCand + 1;

    % ---- lateral profile: quintic blend d0 -> dT over blendLen --------
    u  = min(max((ss - s0)/cfg.local.blendLen, 0), 1);
    bl = 10*u.^3 - 15*u.^4 + 6*u.^5;             % C2 blend
    dd = d0 + (dT - d0).*bl;

    XY = Rxy + Nrm.*dd;

    % ---- geometry ------------------------------------------------------
    [kap, hdg] = pathCurvature(XY, ds);
    if max(abs(kap)) > kappaMax
        continue;                                 % kinematically infeasible
    end

    % ---- static / soft cost along the candidate -----------------------
    sCost = zeros(n,1);
    for i = 1:n
        [cc, rr, ok] = iadpW2G(G, XY(i,1), XY(i,2));
        if ~ok, sCost(i) = 1e4; else, sCost(i) = G.static(rr,cc); end
    end
    if any(sCost > 5e3)
        continue;                                 % leaves the mapped area
    end
    if ~cmd.allowShoulder && mean(sCost) > 0.6*cfg.grid.wEdge
        continue;                                 % shoulder not permitted yet
    end

    % ---- speed profile + collision resolution -------------------------
    % speed cap = behaviour target, comfort lateral acceleration limit
    vCap = min(cmd.vTarget, sqrt(cfg.veh.latAccMax./max(abs(kap), 1e-3)));
    stopIdx = n + 1;
    feasible = false; tt = []; vv = []; colAt = NaN;

    for iter = 1:3
        vv = speedProfile(ss, vCap, ego.v, cfg, stopIdx);
        tt = timeFromSpeed(ss, vv);
        [hit, jHit] = collisionCheck(XY, hdg, tt, preds, cfg);
        if ~hit
            feasible = true; colAt = NaN; break;
        end
        colAt = ss(jHit) - s0;
        % require a full stop a little before the conflict
        margin = max(1, round(1.5/ds));
        newStop = max(1, jHit - margin);
        if newStop >= stopIdx, break; end
        stopIdx = newStop;
        if stopIdx <= 2
            break;                                % cannot stop in time here
        end
    end

    if ~feasible, continue; end
    info.nFeasible = info.nFeasible + 1;

    % ---- cost ----------------------------------------------------------
    dynRisk = riskAlong(XY, G, cfg);
    cost =  cfg.local.wOff    * mean(abs(dd - cmd.latBias)) ...
          + cfg.local.wRisk   * (dynRisk/cfg.grid.wDyn) ...
          + cfg.local.wSmooth * mean(kap.^2) ...
          + cfg.local.wSpeed  * max(0, cmd.vTarget - mean(vv)) ...
          + 0.010             * mean(sCost) ...
          + cfg.local.wJerk   * jerkRMS(vv, tt) ...
          + 0.35              * abs(dT - d0);

    if cost < bestCost
        bestCost = cost;
        best = packTraj(tt, XY, hdg, vv, kap, ss, dd);
        info.dSel = dT; info.collisionAt = colAt;
    end
end

if isempty(best)
    % nothing feasible -> emergency stop on the current heading
    traj = brakeTrajectory(ego, cfg);
    info.fail = true; info.reason = 'no feasible candidate';
    return;
end

traj = best;
info.fail = false;
info.cost = bestCost;
info.reason = 'ok';
end

% =======================================================================
function T = packTraj(t, XY, hdg, v, k, s, d)
T.t = t(:); T.x = XY(:,1); T.y = XY(:,2); T.psi = hdg(:);
T.v = v(:);  T.k = k(:);   T.s = s(:);    T.d = d(:);
end

function T = brakeTrajectory(ego, cfg)
a = cfg.veh.aMin;
v0 = ego.v;
tStop = max(v0/abs(a), 0.05);
t = (0:0.1:max(tStop, 1.5)).';
v = max(0, v0 + a*t);
sdist = v0*t + 0.5*a*t.^2;
sdist(t > tStop) = v0*tStop + 0.5*a*tStop^2;   % hold the stopping point
sdist = max(sdist, 0);
T.t = t;
T.x = ego.x + sdist*cos(ego.psi);
T.y = ego.y + sdist*sin(ego.psi);
T.psi = ego.psi*ones(size(t));
T.v = v;  T.k = zeros(size(t));  T.s = sdist;  T.d = zeros(size(t));
end

% =======================================================================
function [kap, hdg] = pathCurvature(XY, ds)
dx = gradient(XY(:,1), ds); dy = gradient(XY(:,2), ds);
hdg= unwrap(atan2(dy,dx));
ddx= gradient(dx, ds);       ddy= gradient(dy, ds);
kap= (dx.*ddy - dy.*ddx)./max((dx.^2+dy.^2).^1.5, 1e-9);
end

function v = speedProfile(s, vCap, v0, cfg, stopIdx)
n = numel(s);
v = vCap(:);
if numel(v) == 1, v = v*ones(n,1); end
if stopIdx <= n
    v(stopIdx:end) = 0;
end
% backward pass: braking capability
for i = n-1:-1:1
    ds = s(i+1)-s(i);
    v(i) = min(v(i), sqrt(v(i+1)^2 + 2*abs(cfg.veh.aMin)*ds));
end
% forward pass: acceleration capability, anchored at the current speed
v(1) = min(v(1), max(v0, 0));
for i = 2:n
    ds = s(i)-s(i-1);
    v(i) = min(v(i), sqrt(max(v(i-1)^2 + 2*cfg.veh.aMax*ds, 0)));
end
v = max(v, 0);
end

function t = timeFromSpeed(s, v)
n = numel(s); t = zeros(n,1);
for i = 2:n
    vm = max(0.5*(v(i)+v(i-1)), 0.15);
    t(i) = t(i-1) + (s(i)-s(i-1))/vm;
end
end

function [hit, jHit] = collisionCheck(XY, hdg, t, preds, cfg)
hit = false; jHit = NaN;
if isempty(preds), return; end
Lb = cfg.veh.length; Wb = cfg.veh.width;
rEgo = 0.5*hypot(Lb/3, Wb) + cfg.safe.hardBuffer;
for j = 1:numel(t)
    if t(j) > cfg.pred.horizon, break; end
    px = XY(j,1); py = XY(j,2); ph = hdg(j);
    off = [-cfg.veh.rearOH + Lb/6, -cfg.veh.rearOH + Lb/2, -cfg.veh.rearOH + 5*Lb/6];
    for i = 1:numel(preds)
        Pr = preds(i);
        for m = 1:numel(Pr.mode)
            if Pr.mode(m).w < 0.12, continue; end
            xy = interpMode(Pr, m, t(j));
            rr = Pr.r + rEgo;
            for c = 1:3
                cx = px + off(c)*cos(ph); cy = py + off(c)*sin(ph);
                if (cx-xy(1))^2 + (cy-xy(2))^2 < rr^2
                    hit = true; jHit = j; return;
                end
            end
        end
    end
end
end

function xy = interpMode(P, m, t)
if t <= P.t(1)
    a = t/P.t(1);
    xy = P.pos0 + a*(P.mode(m).xy(1,:) - P.pos0);
elseif t >= P.t(end)
    xy = P.mode(m).xy(end,:);
else
    xy = [interp1(P.t, P.mode(m).xy(:,1), t), ...
          interp1(P.t, P.mode(m).xy(:,2), t)];
end
end

function r = riskAlong(XY, G, cfg) %#ok<INUSD>
r = 0;
for i = 1:size(XY,1)
    [cc, rr, ok] = iadpW2G(G, XY(i,1), XY(i,2));
    if ok, r = r + (G.cost(rr,cc) - G.static(rr,cc)); end
end
r = r/max(size(XY,1),1);
end

function j = jerkRMS(v, t)
if numel(v) < 3, j = 0; return; end
a = gradient(v(:), max(t(:),1e-3));
jj= gradient(a, max(t(:),1e-3));
j = sqrt(mean(jj(isfinite(jj)).^2));
if ~isfinite(j), j = 0; end
end
