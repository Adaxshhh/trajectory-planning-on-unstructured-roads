function [fsm, cmd, feat] = iadpBehaviorFSM(fsm, cfg, ego, tracks, preds, map, t, pinfo)
%IADPBEHAVIORFSM  Decision logic (identical semantics to a Stateflow chart).
%
%   [fsm, cmd, feat] = iadpBehaviorFSM(fsm, cfg, ego, tracks, preds, map, t, pinfo)
%
%   States
%     CRUISE     free flow on the route
%     FOLLOW     a slower agent is in the corridor -> match speed + headway
%     OBSTACLE   a static/blocking obstacle -> allow shoulder, plan around
%     YIELD      unsignalised conflict (junction, informal merge) -> give way
%     MERGE      actively closing a gap on a highway merge
%     CREEP      dense market / crowd -> sustained very low speed
%     HOLD       something is in the way that will clear (cattle, crowd)
%     ESTOP      imminent collision -> maximum braking
%
%   The chart is written as an explicit transition list so it can be
%   ported 1:1 into Stateflow; each 'if' below is one transition with the
%   same guard. cfg.fsm.* holds every threshold.
%
%   If Stateflow is installed the same chart can be exercised through
%   iadpToStateflow (see the printed hint), but nothing here depends on it.

if isempty(fsm) || ~isfield(fsm,'state')
    fsm.state = 'CRUISE'; fsm.tIn = 0; fsm.prev = 'CRUISE';
    fsm.holdStart = -inf; fsm.log = {};
end
fsm.tIn = fsm.tIn + 1/cfg.planRate;

%% ---------------- situation features -----------------------------------
feat = features(cfg, ego, tracks, preds, map);
feat.plannerFail = ~isempty(pinfo) && isfield(pinfo,'fail') && pinfo.fail;

s = fsm.state;
prev = s;

%% ---------------- transitions (priority ordered) -----------------------
if feat.ttc < cfg.fsm.ttcBrake || (feat.gapAhead < 2.5 && ego.v > 3)
    s = 'ESTOP';

elseif feat.plannerFail
    s = 'OBSTACLE';

elseif feat.blockingStatic && feat.blockDist < 30
    s = 'OBSTACLE';

elseif feat.vruInCorridor && feat.vruDist < 18
    s = 'HOLD';

elseif feat.conflictAgent && feat.conflictDist < 28
    s = 'YIELD';

elseif feat.density >= cfg.fsm.densityCreep
    s = 'CREEP';

elseif feat.mergeAgent
    s = 'MERGE';

elseif isfinite(feat.leadDist) && feat.leadDist < max(12, cfg.fsm.followGapT*ego.v*1.6)
    s = 'FOLLOW';

else
    s = 'CRUISE';
end

% hysteresis: leave ESTOP / HOLD only when things really clear
if strcmp(fsm.state,'ESTOP') && ~strcmp(s,'ESTOP')
    % stay in ESTOP only while still moving toward a threat; once stopped
    % behind a static blocker fall through so OBSTACLE can steer around it
    if feat.ttc < cfg.fsm.ttcWarn || (feat.gapAhead < 4.0 && ego.v > cfg.fsm.stopDist)
        s = 'ESTOP';
    end
end
if strcmp(fsm.state,'HOLD') && ~strcmp(s,'HOLD') && ~strcmp(s,'ESTOP')
    if feat.vruDist < cfg.fsm.holdClearR && fsm.tIn < 8   % give up waiting after 8 s and creep past
        s = 'HOLD';
    end
end

if ~strcmp(s, fsm.state)
    fsm.log{end+1} = sprintf('t=%6.2f  %-8s -> %-8s  (ttc=%.2f gap=%.1f dens=%d)', ...
        t, fsm.state, s, feat.ttc, feat.gapAhead, feat.density);
    fsm.prev = fsm.state; fsm.state = s; fsm.tIn = 0;
end

%% ---------------- state outputs ----------------------------------------
cmd.latBias       = 0;
cmd.allowShoulder = false;
cmd.mode          = s;

switch s
    case 'CRUISE'
        cmd.vTarget = feat.vRoad;
    case 'FOLLOW'
        vLead = max(feat.leadSpeed, 0);
        dDes  = max(4, cfg.fsm.followGapT*ego.v);
        cmd.vTarget = max(0, min(feat.vRoad, vLead + 0.55*(feat.leadDist - dDes)));
    case 'OBSTACLE'
        cmd.vTarget       = min(feat.vRoad, 5.0);
        cmd.allowShoulder = true;                      % India: use the edge
        cmd.latBias       = feat.freeSide*1.2;
    case 'YIELD'
        cmd.vTarget = cfg.fsm.yieldSpeed;
        if feat.conflictDist < 12, cmd.vTarget = 0.8; end
    case 'MERGE'
        if feat.mergeGapT > cfg.fsm.mergeGapT
            cmd.vTarget = feat.vRoad;                  % take the gap
        else
            cmd.vTarget = max(2, feat.mergeAgentSpeed - 2);  % open one up
            cmd.latBias = -0.6*sign(feat.mergeSide);
        end
        cmd.allowShoulder = false;
    case 'CREEP'
        cmd.vTarget       = cfg.fsm.creepSpeed;
        cmd.allowShoulder = true;
        cmd.latBias       = 0.4*feat.freeSide;
    case 'HOLD'
        cmd.vTarget       = 0;
        cmd.allowShoulder = true;
    case 'ESTOP'
        cmd.vTarget       = 0;
        cmd.allowShoulder = true;
end

cmd.vTarget = max(0, min(cmd.vTarget, feat.vRoad));
cmd.latBias = max(min(cmd.latBias, cfg.local.dMax*0.6), -cfg.local.dMax*0.6);
fsm.cmd = cmd;
fsm.feat = feat;
fsm.changed = ~strcmp(prev, fsm.state);
end

% =======================================================================
function f = features(cfg, ego, tracks, preds, map)
f.ttc = inf; f.gapAhead = inf; f.leadDist = inf; f.leadSpeed = inf;
f.density = 0; f.blockingStatic = false; f.blockDist = inf;
f.vruInCorridor = false; f.vruDist = inf;
f.conflictAgent = false; f.conflictDist = inf;
f.mergeAgent = false; f.mergeGapT = inf; f.mergeAgentSpeed = 0; f.mergeSide = 1;
f.freeSide = 1;
f.vRoad = 13.9;

P = map.route;
[sE, dE, iE] = iadpPathProject(P, ego.x, ego.y);
f.egoS = sE; f.egoD = dE;

% road speed cap from local curvature
kLocal = max(abs(P.curv(max(1,iE-2):min(numel(P.curv),iE+40))));
f.vRoad = min(13.9, sqrt(cfg.veh.latAccMax/max(kLocal,1e-3)));

leftFree = 0; rightFree = 0;
for i = 1:numel(tracks)
    T = tracks(i);
    p = iadpClassParams(T.cls);
    ax = T.x(1); ay = T.x(2); av = T.x(3); apsi = T.x(4);
    d  = hypot(ax-ego.x, ay-ego.y);
    if d < 15, f.density = f.density + 1; end

    [sA, dA] = iadpPathProject(P, ax, ay, iE);
    ds = sA - sE;
    inCorridor = abs(dA) < 2.2 + p.W/2;

    if ds > 0
        if inCorridor
            if ds < f.leadDist
                f.leadDist  = ds - (p.L/2 + cfg.veh.length/2);
                f.leadSpeed = av*cos(wrapA(apsi - P.hdg(min(numel(P.hdg),iE))));
            end
            if av < 0.6 && ds < f.blockDist
                f.blockingStatic = true; f.blockDist = ds;
            end
        end
        if dA > 1.0, leftFree = leftFree + 1/max(ds,1); end
        if dA < -1.0, rightFree = rightFree + 1/max(ds,1); end
    end

    % closing gap along the ego heading
    rel = [ax-ego.x, ay-ego.y];
    fwd = rel*[cos(ego.psi); sin(ego.psi)];
    lat = abs(rel*[-sin(ego.psi); cos(ego.psi)]);
    if fwd > 0 && lat < (cfg.veh.width/2 + p.W/2 + 0.4)
        gap = fwd - (cfg.veh.length/2 + p.L/2);
        f.gapAhead = min(f.gapAhead, max(gap,0));
        vClose = ego.v - av*cos(wrapA(apsi - ego.psi));
        if vClose > 0.2
            f.ttc = min(f.ttc, max(gap,0)/vClose);
        end
    end

    % lateral conflict (junction / informal merge)
    if ~p.laneBound || abs(wrapA(apsi - ego.psi)) > deg2rad(35)
        tca = closestApproach(ego, T, cfg.pred.horizon);
        if tca.dmin < (cfg.veh.width/2 + p.r + 0.6) && tca.t > 0
            f.conflictAgent = true;
            f.conflictDist  = min(f.conflictDist, d);
        end
    end

    % merging vehicle coming from the side at similar heading
    if p.laneBound && abs(wrapA(apsi - ego.psi)) < deg2rad(40) && ...
       abs(dA) > 1.5 && abs(dA) < 7 && ds > -10 && ds < 45
        f.mergeAgent = true;
        f.mergeAgentSpeed = av;
        f.mergeSide = sign(dA);
        f.mergeGapT = abs(ds)/max(abs(ego.v - av), 0.5);
    end
end

% vulnerable road users predicted to enter the corridor
for i = 1:numel(preds)
    Pr = preds(i);
    p  = iadpClassParams(Pr.cls);
    if ~p.vru, continue; end
    for m = 1:numel(Pr.mode)
        if Pr.mode(m).w < 0.15, continue; end
        xy = Pr.mode(m).xy;
        for k = 1:size(xy,1)
            [sA, dA] = iadpPathProject(P, xy(k,1), xy(k,2), iE);
            if abs(dA) < 2.2 && sA > sE - 2 && sA - sE < 25
                f.vruInCorridor = true;
                f.vruDist = min(f.vruDist, hypot(Pr.pos0(1)-ego.x, Pr.pos0(2)-ego.y));
                break;
            end
        end
    end
end

f.freeSide = 1;
if rightFree < leftFree, f.freeSide = -1; end
end

% =======================================================================
function r = closestApproach(ego, T, H)
% constant-velocity closest point of approach
pe = [ego.x ego.y]; ve = ego.v*[cos(ego.psi) sin(ego.psi)];
pa = [T.x(1) T.x(2)]; va = T.x(3)*[cos(T.x(4)) sin(T.x(4))];
dp = pa - pe; dv = va - ve;
den = dv*dv.';
if den < 1e-6
    r.t = 0; r.dmin = norm(dp); return;
end
tc = -(dp*dv.')/den;
tc = max(0, min(tc, H));
r.t = tc;
r.dmin = norm(dp + dv*tc);
end

function a = wrapA(a)
a = mod(a + pi, 2*pi) - pi;
end
