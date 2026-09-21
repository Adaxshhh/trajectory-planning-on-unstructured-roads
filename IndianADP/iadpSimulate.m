function res = iadpSimulate(cfg, S, opts)
%IADPSIMULATE  Closed-loop run of one scenario.
%
%   res = iadpSimulate(cfg, S)
%   res = iadpSimulate(cfg, S, opts)     opts.live (logical), opts.quiet
%
%   Pipeline executed every planning cycle (cfg.planRate Hz):
%
%     sensors -> fusion/tracking -> classification -> prediction
%             -> risk grid -> behaviour FSM -> local planner
%             -> (hybrid A* fallback) -> controller -> vehicle model
%
%   The vehicle model runs at cfg.dt with zero-order-hold on the control,
%   which is exactly how the Simulink model would be wired.
%
%   res contains the full log used by iadpMetrics and iadpVisualize.

if nargin < 3, opts = struct; end
if ~isfield(opts,'live'),  opts.live  = false; end
if ~isfield(opts,'quiet'), opts.quiet = ~cfg.io.verbose; end

rng(cfg.rngSeed + numel(S.name));

ego    = S.ego0;
actors = S.actors;
map    = S.map;
trk    = iadpFusion('init', cfg);
fsm    = [];
ctl    = struct('eInt',0);
traj   = [];
cmd    = struct('vTarget',S.ego0.v,'latBias',0,'allowShoulder',false,'mode','CRUISE');

refPath   = map.route;
refIsAStar= false;
failCount = 0;

dt        = cfg.dt;
planEvery = max(1, round(1/(cfg.planRate*dt)));
nSteps    = round(S.meta.tMax/dt);

% ---- logs --------------------------------------------------------------
L.t = zeros(nSteps,1); L.x = L.t; L.y = L.t; L.psi = L.t; L.v = L.t;
L.a = L.t; L.delta = L.t; L.curv = L.t; L.s = L.t; L.d = L.t;
L.state = cell(nSteps,1);
L.minClear = inf(nSteps,1);
L.nTracks = zeros(nSteps,1);
L.nDets   = zeros(nSteps,1);
L.ttc     = inf(nSteps,1);
L.onShoulder = false(nSteps,1);
L.latency = [];  L.latencyT = [];
L.astar   = [];  L.astarT = [];
L.planFail= 0;   L.astarCalls = 0; L.astarSuccess = 0;
L.classOK = []; L.classN = 0; L.classHit = 0;
frames = {};

collision  = false; collisionInfo = struct('t',NaN,'with','');
goalReached= false; goalTime = NaN;
stuckTimer = 0; stuckFlag = false;

for k = 1:nSteps
    t = (k-1)*dt;

    %% ================= planning cycle ==================================
    if mod(k-1, planEvery) == 0
        tPlan = tic;

        dets = iadpSensors(cfg, ego, actors, t);
        [trk, tracks] = iadpFusion('update', trk, dets, planEvery*dt, cfg);
        [cHit, cN]    = classAccuracy(tracks, actors);
        L.classHit = L.classHit + cHit; L.classN = L.classN + cN;

        preds = iadpPredict(tracks, cfg, map, ego);
        G     = iadpRiskGrid(map, cfg, ego, preds);

        [fsm, cmd] = iadpBehaviorFSM(fsm, cfg, ego, tracks, preds, map, t, ...
                                     struct('fail', failCount>0));

        [trajNew, pinfo] = iadpLocalPlanner(refPath, ego, preds, G, cfg, cmd);

        if pinfo.fail
            failCount = failCount + 1;
            L.planFail = L.planFail + 1;
        else
            failCount = 0;
            traj = trajNew;
        end

        %% ---- hybrid A* fallback / unstructured replanning -------------
        needAStar = failCount >= cfg.local.failsToAStar || ...
                    (strcmp(cmd.mode,'OBSTACLE') && ~refIsAStar && fsm.changed);
        if needAStar
            [sE, ~, iE] = iadpPathProject(map.route, ego.x, ego.y);
            sG = min(sE + cfg.hybrid.goalAhead, map.route.len);
            iG = find(map.route.s >= sG, 1, 'first');
            if isempty(iG), iG = numel(map.route.s); end
            goalPose = [map.route.xy(iG,1) map.route.xy(iG,2) map.route.hdg(iG)];

            L.astarCalls = L.astarCalls + 1;
            [apath, ainfo] = iadpHybridAStar(G, cfg, [ego.x ego.y ego.psi], goalPose);
            L.astar(end+1)  = ainfo.time; %#ok<AGROW>
            L.astarT(end+1) = t;          %#ok<AGROW>
            if ~isempty(apath) && size(apath,1) > 3
                L.astarSuccess = L.astarSuccess + 1;
                refPath    = iadpMakePath(apath(:,1:2), 0.5);
                refIsAStar = true;
                failCount  = 0;
                cmd.latBias = 0;
                [trajNew, pinfo] = iadpLocalPlanner(refPath, ego, preds, G, cfg, cmd);
                if ~pinfo.fail, traj = trajNew; end
            end
            if iE < 1, iE = 1; end %#ok<NASGU>
        end

        % rejoin the route once the detour path is nearly consumed
        if refIsAStar
            [sR, ~] = iadpPathProject(refPath, ego.x, ego.y);
            if refPath.len - sR < 6
                refPath = map.route; refIsAStar = false;
            end
        end

        lat = toc(tPlan);
        L.latency(end+1)  = lat; %#ok<AGROW>
        L.latencyT(end+1) = t;   %#ok<AGROW>

        if opts.live || (cfg.io.video && mod(k-1, planEvery*2) == 0)
            frames{end+1} = snapshot(t, ego, actors, tracks, preds, traj, ...
                                     refPath, cmd, fsm, G); %#ok<AGROW>
        end
    end

    %% ================= control + plant =================================
    [delta, aCmd, ctl] = iadpController(ego, traj, cfg, ctl, dt);
    ego = iadpVehicleStep(ego, delta, aCmd, cfg, dt);
    actors = iadpStepActors(actors, dt, t, ego, map);

    %% ================= logging / events ================================
    [sE, dE] = iadpPathProject(map.route, ego.x, ego.y);
    L.t(k)=t; L.x(k)=ego.x; L.y(k)=ego.y; L.psi(k)=ego.psi; L.v(k)=ego.v;
    L.a(k)=ego.a; L.delta(k)=ego.delta;
    L.curv(k) = tan(ego.delta)/cfg.veh.L;
    L.s(k)=sE; L.d(k)=dE;
    L.state{k} = cmd.mode;
    L.nTracks(k) = numel(trk.tracks);

    [mc, who, ttc] = clearanceAndTTC(ego, actors, cfg);
    L.minClear(k) = mc;  L.ttc(k) = ttc;

    [cc, rr, okg] = iadpW2G(map, ego.x, ego.y);
    L.onShoulder(k) = okg && map.soft(rr,cc);

    if mc <= 0 && ~collision
        collision = true;
        collisionInfo.t = t; collisionInfo.with = who;
        if ~opts.quiet
            fprintf('[iadp]   !! COLLISION at t=%.2f s with %s\n', t, who);
        end
    end

    if ego.v < cfg.fsm.stopDist
        stuckTimer = stuckTimer + dt;
    else
        stuckTimer = 0;
    end
    if stuckTimer > 15, stuckFlag = true; end

    if ~goalReached && sE >= S.goal.s - S.goal.tol
        goalReached = true; goalTime = t;
        nSteps = k;                       % truncate logs
        break;
    end
end

%% ---- trim logs --------------------------------------------------------
n = max(1, min(k, numel(L.t)));
f = {'t','x','y','psi','v','a','delta','curv','s','d','minClear','nTracks','ttc','onShoulder'};
for i = 1:numel(f), L.(f{i}) = L.(f{i})(1:n); end
L.state = L.state(1:n);

res.name     = S.name;
res.desc     = S.meta.desc;
res.cfg      = cfg;
res.log      = L;
res.frames   = frames;
res.map      = map;
res.actors   = actors;
res.goal     = S.goal;
res.collision= collision;
res.collisionInfo = collisionInfo;
res.goalReached = goalReached;
res.goalTime = goalTime;
res.stuck    = stuckFlag;
res.fsmLog   = fsm.log;
res.checks   = S.meta.checks;
res.tEnd     = L.t(end);

if ~opts.quiet
    fprintf('[iadp]   finished: goal=%d collision=%d t=%.1fs replans=%d A*=%d/%d\n', ...
        goalReached, collision, res.tEnd, numel(L.latency), L.astarSuccess, L.astarCalls);
end
end

% =======================================================================
function F = snapshot(t, ego, actors, tracks, preds, traj, refPath, cmd, fsm, G)
F.t = t;
F.ego = [ego.x ego.y ego.psi ego.v];
A = zeros(numel(actors),5); cls = cell(numel(actors),1);
for i = 1:numel(actors)
    A(i,:) = [actors(i).x actors(i).y actors(i).psi actors(i).p.L actors(i).p.W];
    cls{i} = actors(i).cls;
end
F.actors = A; F.actorCls = cls;
T = zeros(numel(tracks),4); tcls = cell(numel(tracks),1);
for i = 1:numel(tracks)
    T(i,:) = [tracks(i).x(1) tracks(i).x(2) tracks(i).x(3) tracks(i).x(4)];
    tcls{i} = tracks(i).cls;
end
F.tracks = T; F.trackCls = tcls;
if ~isempty(traj), F.traj = [traj.x traj.y]; else, F.traj = []; end
F.ref = refPath.xy;
F.mode = cmd.mode;
F.vTarget = cmd.vTarget;
F.state = fsm.state;
% coarse risk field for display (every 4th cell)
F.riskX = G.xmin; F.riskY = G.ymin; F.riskRes = G.res*4;
F.risk = G.cost(1:4:end, 1:4:end);
P = [];
for i = 1:numel(preds)
    for m = 1:numel(preds(i).mode)
        if preds(i).mode(m).w > 0.2
            P = [P; preds(i).mode(m).xy; NaN NaN]; %#ok<AGROW>
        end
    end
end
F.pred = P;
end

% =======================================================================
function [mc, who, ttc] = clearanceAndTTC(ego, actors, cfg)
mc = inf; who = ''; ttc = inf;
ep = [ego.x + (cfg.veh.length/2 - cfg.veh.rearOH)*cos(ego.psi), ...
      ego.y + (cfg.veh.length/2 - cfg.veh.rearOH)*sin(ego.psi), ego.psi];
for i = 1:numel(actors)
    a = actors(i);
    if ~a.alive, continue; end
    d = iadpRectDist(ep, [cfg.veh.length cfg.veh.width], ...
                     [a.x a.y a.psi], [a.p.L a.p.W]);
    if d < mc, mc = d; who = sprintf('%s#%d', a.cls, a.id); end

    rel = [a.x-ego.x, a.y-ego.y];
    fwd = rel*[cos(ego.psi); sin(ego.psi)];
    lat = abs(rel*[-sin(ego.psi); cos(ego.psi)]);
    if fwd > 0 && lat < (cfg.veh.width/2 + a.p.W/2 + 0.3)
        vClose = ego.v - a.v*cos(a.psi - ego.psi);
        if vClose > 0.2
            ttc = min(ttc, max(d,0)/vClose);
        end
    end
end
end

% =======================================================================
function [hit, n] = classAccuracy(tracks, actors)
hit = 0; n = 0;
for i = 1:numel(tracks)
    T = tracks(i);
    best = inf; bc = '';
    for j = 1:numel(actors)
        d = hypot(actors(j).x - T.x(1), actors(j).y - T.x(2));
        if d < best, best = d; bc = actors(j).cls; end
    end
    if best < 2.5
        n = n + 1;
        hit = hit + strcmp(bc, T.cls);
    end
end
end
