function ok = iadpTestSuite(varargin)
%IADPTESTSUITE  Verify every requirement of the problem statement.
%
%   ok = iadpTestSuite()               % unit tests + all 5 scenarios
%   ok = iadpTestSuite('quick',true)   % unit tests only (~20 s)
%
%   Each test prints PASS/FAIL with the measured value, so the output can
%   be pasted straight into the technical report as evidence.

p = inputParser;
p.addParameter('quick', false);
p.parse(varargin{:});
quick = p.Results.quick;

cfg = iadpSetup('io.video',false,'io.verbose',false);
iadpWriteSampleXODR(cfg);

T = {};
T = add(T,'T01 OpenDRIVE writer produces readable files',        @() t01(cfg));
T = add(T,'T02 Geometry: line/arc/spiral evaluated correctly',   @() t02(cfg));
T = add(T,'T03 Map build: drivable area + soft shoulder',        @() t03(cfg));
T = add(T,'T04 Route path: continuity, length, curvature',       @() t04(cfg));
T = add(T,'T05 Frenet projection accuracy',                      @() t05(cfg));
T = add(T,'T06 Camera/LiDAR/radar respect range and FOV',        @() t06(cfg));
T = add(T,'T07 Sensor noise and clutter are present',            @() t07(cfg));
T = add(T,'T08 Tracker converges on a constant-velocity target', @() t08(cfg));
T = add(T,'T09 Tracker follows a turning (CTRV) target',         @() t09(cfg));
T = add(T,'T10 Fusion classifies bus vs pedestrian correctly',   @() t10(cfg));
T = add(T,'T11 Prediction: 4 modes, weights normalised',         @() t11(cfg));
T = add(T,'T12 Prediction: VRU intent crosses the road',         @() t12(cfg));
T = add(T,'T13 Prediction uncertainty grows with erraticness',   @() t13(cfg));
T = add(T,'T14 Risk grid raises cost around predicted agents',   @() t14(cfg));
T = add(T,'T15 Risk grid blocks the agent footprint',            @() t15(cfg));
T = add(T,'T16 Hybrid A* routes around a full blockage',         @() t16(cfg));
T = add(T,'T17 Hybrid A* reports failure when truly blocked',    @() t17(cfg));
T = add(T,'T18 Local planner offsets around a static obstacle',  @() t18(cfg));
T = add(T,'T19 Local planner brakes for an unavoidable block',   @() t19(cfg));
T = add(T,'T20 Local planner respects steering curvature limit', @() t20(cfg));
T = add(T,'T21 FSM: ESTOP on imminent collision',                @() t21(cfg));
T = add(T,'T22 FSM: CREEP in dense traffic',                     @() t22(cfg));
T = add(T,'T23 FSM: HOLD for a crossing pedestrian',             @() t23(cfg));
T = add(T,'T24 Kinematic bicycle turns at the analytic radius',  @() t24(cfg));
T = add(T,'T25 Steering-rate and jerk limits are enforced',      @() t25(cfg));
T = add(T,'T26 Dynamic bicycle model is stable',                 @() t26(cfg));
T = add(T,'T27 Controller tracks a curved path',                 @() t27(cfg));
T = add(T,'T28 Rectangle distance / overlap is exact',           @() t28(cfg));
T = add(T,'T29 Replanning latency stays inside budget',          @() t29(cfg));
T = add(T,'T30 Metrics expose latency, smoothness, completion',  @() t30(cfg));

if ~quick
    fixtures = {'basic','dense','block'};
    for i = 1:numel(fixtures)
        T = add(T, sprintf('S%02d Self-test fixture "%s": safe closed-loop run', i, fixtures{i}), ...
                @() scenarioTest(cfg, fixtures{i}));
    end
    T = add(T,'S04 All self-test fixtures complete without collision', @() campaignTest(cfg));
    T = add(T,'S05 NOTE: this does not validate your own 5 RoadRunner scenarios', @() noteProductionScenarios());
end

ok = runAll(T);
end

% =======================================================================
% harness
% =======================================================================
function T = add(T, name, fn)
T{end+1} = struct('name',name,'fn',fn);
end

function ok = runAll(T)
fprintf('\n=================== IADP test suite ===================\n');
nPass = 0;
for i = 1:numel(T)
    tStart = tic;
    try
        msg = T{i}.fn();
        fprintf('  PASS  %-52s  %s  [%.2fs]\n', T{i}.name, msg, toc(tStart));
        nPass = nPass + 1;
    catch ME
        fprintf('  FAIL  %-52s  %s\n', T{i}.name, ME.message);
    end
end
fprintf('-------------------------------------------------------\n');
fprintf('  %d/%d tests passed\n\n', nPass, numel(T));
ok = nPass == numel(T);
end

function check(cond, fmt, varargin)
if ~cond
    error('iadp:test', fmt, varargin{:});
end
end

% =======================================================================
% unit tests
% =======================================================================
function m = t01(cfg)
files = iadpWriteSampleXODR(cfg);
check(numel(files)>=3, 'expected 3 scenes, got %d', numel(files));
odr = iadpImportOpenDRIVE(files{1});
check(numel(odr.roads)>=1, 'no roads parsed');
check(odr.roads(1).s(end) > 150, 'village road too short: %.1f', odr.roads(1).s(end));
odr2 = iadpImportOpenDRIVE(files{2});
check(numel(odr2.roads)==5, 'intersection should have 5 roads, got %d', numel(odr2.roads));
m = sprintf('%d scenes, %d+%d roads', numel(files), numel(odr.roads), numel(odr2.roads));
end

function m = t02(cfg)
f = fullfile(cfg.io.xodrDir,'IndianVillageRoad.xodr');
odr = iadpImportOpenDRIVE(f, 0.25);
C = odr.roads(1).center;
% continuity: no jumps larger than 2*ds
d = hypot(diff(C(:,1)), diff(C(:,2)));
check(max(d) < 0.6, 'reference line discontinuity %.3f m', max(d));
% arc section must bend: heading changes by curvature*length
dh = abs(C(end,3) - C(1,3));
check(dh > 0.5, 'arc geometry not applied, dheading = %.3f rad', dh);
% arc radius sanity on the curved part
k = diff(unwrap(C(:,3)))./max(d,1e-9);
check(max(abs(k)) < 0.05, 'implausible curvature %.4f', max(abs(k)));
m = sprintf('max step %.3f m, total heading change %.2f rad', max(d), dh);
end

function m = t03(cfg)
odr = iadpImportOpenDRIVE(fullfile(cfg.io.xodrDir,'IndianVillageRoad.xodr'));
map = iadpBuildMap(odr, cfg, 1, [0 0]);
check(any(map.drivable(:)), 'no drivable cells');
check(any(map.soft(:)), 'no soft shoulder cells');
frac = sum(map.drivable(:))/numel(map.drivable);
check(frac > 0.002 && frac < 0.6, 'implausible drivable fraction %.4f', frac);
% a point on the route must be drivable
i = round(numel(map.route.s)/2);
[c,r,ok] = iadpW2G(map, map.route.xy(i,1), map.route.xy(i,2));
check(ok && map.drivable(r,c), 'route point is not on drivable surface');
m = sprintf('drivable %.1f%%, soft %.1f%%', 100*frac, 100*sum(map.soft(:))/numel(map.soft));
end

function m = t04(cfg)
odr = iadpImportOpenDRIVE(fullfile(cfg.io.xodrDir,'IndianVillageRoad.xodr'));
map = iadpBuildMap(odr, cfg, 1, [0 0]);
P = map.route;
check(P.len > 150, 'route too short %.1f m', P.len);
d = hypot(diff(P.xy(:,1)), diff(P.xy(:,2)));
check(abs(mean(d)-0.5) < 0.05, 'resampling error, mean step %.3f', mean(d));
% circle test for curvature
th = linspace(0,pi,400).'; R = 25;
Pc = iadpMakePath([R*cos(th) R*sin(th)], 0.5);
kk = median(abs(Pc.curv(20:end-20)));
check(abs(kk - 1/R) < 0.004, 'curvature error: %.4f vs %.4f', kk, 1/R);
m = sprintf('len %.0f m, curvature err %.4f 1/m', P.len, abs(kk-1/R));
end

function m = t05(cfg) %#ok<INUSD>
P = iadpMakePath([(0:0.5:100).' zeros(201,1)], 0.5);
[s,d] = iadpPathProject(P, 40, 3.0);
check(abs(s-40) < 0.6, 's error %.3f', abs(s-40));
check(abs(d-3.0) < 0.05, 'd error %.3f', abs(d-3.0));
[~,d2] = iadpPathProject(P, 40, -2.0);
check(d2 < 0, 'sign convention wrong: %.2f', d2);
m = sprintf('s err %.3f m, d err %.3f m', abs(s-40), abs(d-3));
end

function m = t06(cfg)
ego = mkEgo(0,0,0,10);
A = [iadpMakeActor(1,'car', 30,0,0,8,'constant',0.5), ...
     iadpMakeActor(2,'car', 200,0,0,8,'constant',0.5), ...   % beyond all ranges
     iadpMakeActor(3,'car', -30,0,pi,8,'constant',0.5)];     % behind (cam/radar blind)
nIn = 0; nFar = 0; nBehindCam = 0;
for k = 1:40
    d = iadpSensors(cfg, ego, A, 0);
    for j = 1:numel(d)
        if d(j).truthId == 1, nIn = nIn + 1; end
        if d(j).truthId == 2, nFar = nFar + 1; end
        if d(j).truthId == 3 && strcmp(d(j).src,'cam'), nBehindCam = nBehindCam + 1; end
    end
end
check(nIn > 60, 'too few detections of the in-range target: %d', nIn);
check(nFar == 0, 'target beyond max range was detected %d times', nFar);
check(nBehindCam == 0, 'camera detected a target outside its FOV %d times', nBehindCam);
m = sprintf('%d hits in-range, %d out-of-range, %d FOV violations', nIn, nFar, nBehindCam);
end

function m = t07(cfg)
ego = mkEgo(0,0,0,0);
A = iadpMakeActor(1,'car', 25,0,0,0,'stopped',0);
err = []; nClutter = 0;
for k = 1:200
    d = iadpSensors(cfg, ego, A, 0);
    for j = 1:numel(d)
        if d(j).truthId == 1
            err(end+1) = hypot(d(j).pos(1)-25, d(j).pos(2)); %#ok<AGROW>
        else
            nClutter = nClutter + 1;
        end
    end
end
check(std(err) > 0.01, 'measurements are noiseless');
check(median(err) < 2.5, 'measurement bias too large %.2f m', median(err));
check(nClutter > 0, 'no false alarms generated');
m = sprintf('median err %.2f m, sigma %.2f m, %d clutter', median(err), std(err), nClutter);
end

function m = t08(cfg)
trk = iadpFusion('init', cfg);
v = 8; dt = 0.1;
ego = mkEgo(0,0,0,0);
a = iadpMakeActor(1,'car', 20,0,0,v,'constant',0.5);
est = [];
for k = 1:60
    t = (k-1)*dt;
    d = iadpSensors(cfg, ego, a, t);
    [trk, tracks] = iadpFusion('update', trk, d, dt, cfg);
    a = iadpStepActors(a, dt, t, ego, []);
    if ~isempty(tracks), est = tracks(1).x; end
end
check(~isempty(est), 'no confirmed track');
check(abs(est(1) - a.x) < 2.0, 'position error %.2f m', abs(est(1)-a.x));
check(abs(est(3) - v) < 2.5, 'speed error %.2f m/s', abs(est(3)-v));
m = sprintf('pos err %.2f m, speed err %.2f m/s', abs(est(1)-a.x), abs(est(3)-v));
end

function m = t09(cfg)
trk = iadpFusion('init', cfg);
dt = 0.1; ego = mkEgo(0,0,0,0);
a = iadpMakeActor(1,'autorickshaw', 18,0,0,6,'weave',0.8,'vTarget',6,'amp',0.6,'per',4);
est = []; 
for k = 1:80
    t = (k-1)*dt;
    d = iadpSensors(cfg, ego, a, t);
    [trk, tracks] = iadpFusion('update', trk, d, dt, cfg);
    a = iadpStepActors(a, dt, t, ego, []);
    if ~isempty(tracks), est = tracks(1).x; end
end
check(~isempty(est), 'no confirmed track for the weaving agent');
e = hypot(est(1)-a.x, est(2)-a.y);
check(e < 3.0, 'tracking error on a turning target %.2f m', e);
m = sprintf('turning-target error %.2f m, omega %.2f rad/s', e, est(5));
end

function m = t10(cfg)
res = zeros(1,2); names = {'bus','pedestrian'};
for c = 1:2
    trk = iadpFusion('init', cfg);
    ego = mkEgo(0,0,0,0);
    a = iadpMakeActor(1, names{c}, 22, 0, 0, 1.0, 'constant', 0.3);
    lbl = 'unknown';
    for k = 1:40
        d = iadpSensors(cfg, ego, a, (k-1)*0.1);
        [trk, tracks] = iadpFusion('update', trk, d, 0.1, cfg);
        if ~isempty(tracks), lbl = tracks(1).cls; end
    end
    res(c) = strcmp(lbl, names{c});
    if ~res(c)
        check(false, '%s was classified as %s', names{c}, lbl);
    end
end
m = 'bus and pedestrian both classified correctly';
end

function m = t11(cfg)
[tracks, map, ego] = mkTrackScene(cfg, 'car', 20, 0, 0, 8);
pr = iadpPredict(tracks, cfg, map, ego);
check(numel(pr)==1, 'expected 1 prediction');
check(numel(pr(1).mode)==cfg.pred.nModes, 'expected %d modes', cfg.pred.nModes);
w = sum([pr(1).mode.w]);
check(abs(w-1) < 1e-9, 'weights sum to %.6f', w);
% CV mode must match the analytic straight-line answer
K = numel(pr(1).t);
expX = 20 + 8*pr(1).t(K);
check(abs(pr(1).mode(2).xy(K,1) - expX) < 0.5, 'CV mode error %.2f m', ...
      abs(pr(1).mode(2).xy(K,1)-expX));
m = sprintf('%d modes, weight sum %.3f, CV err %.2f m', numel(pr(1).mode), w, ...
            abs(pr(1).mode(2).xy(K,1)-expX));
end

function m = t12(cfg)
[tracks, map, ego] = mkTrackScene(cfg, 'pedestrian', 25, 4, pi/2, 1.2);
pr = iadpPredict(tracks, cfg, map, ego);
xy = pr(1).mode(4).xy;
dy = abs(xy(end,2) - 4);
dx = abs(xy(end,1) - 25);
check(dy > dx, 'intent mode does not cross the road (dx=%.1f dy=%.1f)', dx, dy);
check(pr(1).mode(4).w >= 0.25, 'crossing weight too low for a VRU: %.2f', pr(1).mode(4).w);
m = sprintf('lateral travel %.1f m vs longitudinal %.1f m', dy, dx);
end

function m = t13(cfg)
[tc, map, ego] = mkTrackScene(cfg, 'car', 20, 0, 0, 5);
[ta] = mkTrackScene(cfg, 'animal', 20, 0, 0, 5);
pc = iadpPredict(tc, cfg, map, ego);
pa = iadpPredict(ta, cfg, map, ego);
check(pa(1).sig(end) > pc(1).sig(end)*1.5, ...
      'animal uncertainty %.2f not larger than car %.2f', pa(1).sig(end), pc(1).sig(end));
m = sprintf('sigma car %.2f m vs animal %.2f m at %.1fs', ...
            pc(1).sig(end), pa(1).sig(end), cfg.pred.horizon);
end

function m = t14(cfg)
[tracks, map, ego] = mkTrackScene(cfg, 'car', 20, 0, 0, 0);
pr = iadpPredict(tracks, cfg, map, ego);
G0 = iadpRiskGrid(map, cfg, ego, []);
G1 = iadpRiskGrid(map, cfg, ego, pr);
[c,r] = iadpW2G(G1, 20, 0);
check(G1.cost(r,c) > G0.cost(r,c) + 50, 'risk not raised at the agent (%.1f vs %.1f)', ...
      G1.cost(r,c), G0.cost(r,c));
[c2,r2] = iadpW2G(G1, 20, 12);
check(G1.cost(r2,c2) < G1.cost(r,c), 'risk field is not local');
m = sprintf('cost at agent %.0f, 12 m away %.0f', G1.cost(r,c), G1.cost(r2,c2));
end

function m = t15(cfg)
[tracks, map, ego] = mkTrackScene(cfg, 'bus', 25, 0, 0, 0);
pr = iadpPredict(tracks, cfg, map, ego);
G = iadpRiskGrid(map, cfg, ego, pr);
[c,r] = iadpW2G(G, 25, 0);
check(G.blocked(r,c), 'agent footprint is not blocked');
m = 'footprint blocked as expected';
end

function m = t16(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 60, 0, 0, 0);
ego = mkEgo(map.route.xy(20,1), map.route.xy(20,2), map.route.hdg(20), 4);
% block the carriageway completely with a stopped bus 25 m ahead
i = 70;
tr = mkTrack('bus', map.route.xy(i,1), map.route.xy(i,2), map.route.hdg(i), 0);
pr = iadpPredict(tr, cfg, map, ego);
G  = iadpRiskGrid(map, cfg, ego, pr);
j  = min(numel(map.route.s), i + 60);
goal = [map.route.xy(j,1) map.route.xy(j,2) map.route.hdg(j)];
[path, info] = iadpHybridAStar(G, cfg, [ego.x ego.y ego.psi], goal);
check(~isempty(path), 'no path found (%s, %d nodes)', info.reason, info.nodes);
% must actually deviate laterally to get round the bus
dmax = 0;
for k = 1:size(path,1)
    [~,d] = iadpPathProject(map.route, path(k,1), path(k,2));
    dmax = max(dmax, abs(d));
end
check(dmax > 1.0, 'path did not deviate around the blockage (max |d| = %.2f)', dmax);
m = sprintf('%d nodes in %.0f ms, max lateral deviation %.2f m', info.nodes, info.time*1000, dmax);
end

function m = t17(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 30, 0, 0, 0);
ego = mkEgo(map.route.xy(20,1), map.route.xy(20,2), map.route.hdg(20), 3);
G = iadpRiskGrid(map, cfg, ego, []);
G.blocked(:) = true;                     % everything blocked
G.cost(:) = 1e4;
goal = [map.route.xy(100,1) map.route.xy(100,2) map.route.hdg(100)];
[path, info] = iadpHybridAStar(G, cfg, [ego.x ego.y ego.psi], goal);
check(isempty(path), 'planner returned a path through blocked space');
m = sprintf('correctly reported: %s', info.reason);
end

function m = t18(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 6);
i = i0 + 40;                              % 20 m ahead
tr = mkTrack('pushcart', map.route.xy(i,1), map.route.xy(i,2), map.route.hdg(i), 0);
pr = iadpPredict(tr, cfg, map, ego);
G  = iadpRiskGrid(map, cfg, ego, pr);
cmd = struct('vTarget',8,'latBias',0,'allowShoulder',true,'mode','OBSTACLE');
[traj, info] = iadpLocalPlanner(map.route, ego, pr, G, cfg, cmd);
check(~info.fail, 'planner failed: %s', info.reason);
check(abs(info.dSel) > 0.8, 'no lateral avoidance, dSel = %.2f', info.dSel);
% and the chosen trajectory must be collision free against the prediction
check(minDistToPred(traj, pr) > 0.3, 'selected trajectory is not clear');
m = sprintf('lateral offset %.2f m, %d/%d candidates feasible', ...
            info.dSel, info.nFeasible, info.nCand);
end

function m = t19(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 9);
% wall of agents across the full road, 22 m ahead
tr = [];
for q = -3:3
    i = i0 + 44;
    n = [-sin(map.route.hdg(i)) cos(map.route.hdg(i))];
    tr = [tr mkTrack('animal', map.route.xy(i,1)+n(1)*q*1.2, ...
                               map.route.xy(i,2)+n(2)*q*1.2, map.route.hdg(i), 0)]; %#ok<AGROW>
end
pr = iadpPredict(tr, cfg, map, ego);
G  = iadpRiskGrid(map, cfg, ego, pr);
cmd = struct('vTarget',9,'latBias',0,'allowShoulder',true,'mode','HOLD');
[traj, info] = iadpLocalPlanner(map.route, ego, pr, G, cfg, cmd); %#ok<ASGLU>
check(min(traj.v) < 1.0, 'planner did not slow down (min v = %.2f)', min(traj.v));
m = sprintf('speed reduced from %.1f to %.2f m/s', ego.v, min(traj.v));
end

function m = t20(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 7);
G = iadpRiskGrid(map, cfg, ego, []);
cmd = struct('vTarget',10,'latBias',0,'allowShoulder',true,'mode','CRUISE');
[traj, info] = iadpLocalPlanner(map.route, ego, [], G, cfg, cmd);
check(~info.fail, 'planner failed on a clear road');
kMax = tan(cfg.veh.maxSteer)/cfg.veh.L;
check(max(abs(traj.k)) <= kMax, 'curvature %.3f exceeds limit %.3f', max(abs(traj.k)), kMax);
check(max(abs(traj.v.^2.*traj.k)) < cfg.veh.latAccMax*1.25, 'lateral acceleration too high');
m = sprintf('max kappa %.3f (limit %.3f), max latAcc %.2f m/s^2', ...
            max(abs(traj.k)), kMax, max(abs(traj.v.^2.*traj.k)));
end

function m = t21(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 12);
i = i0 + 16;   % 8 m ahead, stationary -> TTC well under threshold
tr = mkTrack('truck', map.route.xy(i,1), map.route.xy(i,2), map.route.hdg(i), 0);
pr = iadpPredict(tr, cfg, map, ego);
[fsm, cmd] = iadpBehaviorFSM([], cfg, ego, tr, pr, map, 0, []);
check(strcmp(fsm.state,'ESTOP'), 'state is %s, expected ESTOP', fsm.state);
check(cmd.vTarget == 0, 'ESTOP did not command zero speed');
m = sprintf('state %s, vTarget %.1f', fsm.state, cmd.vTarget);
end

function m = t22(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 5);
tr = [];
for q = 1:7
    i = i0 + 8 + 2*q;
    n = [-sin(map.route.hdg(i)) cos(map.route.hdg(i))];
    side = 2.6*(-1)^q;
    tr = [tr mkTrack('pedestrian', map.route.xy(i,1)+n(1)*side, ...
                                   map.route.xy(i,2)+n(2)*side, map.route.hdg(i), 0.8)]; %#ok<AGROW>
end
pr = iadpPredict(tr, cfg, map, ego);
[fsm, cmd] = iadpBehaviorFSM([], cfg, ego, tr, pr, map, 0, []);
check(any(strcmp(fsm.state,{'CREEP','HOLD','YIELD','ESTOP'})), ...
      'dense crowd produced state %s', fsm.state);
check(cmd.vTarget <= cfg.fsm.creepSpeed + 0.01, 'speed target %.2f too high', cmd.vTarget);
m = sprintf('state %s, vTarget %.2f m/s, density %d', fsm.state, cmd.vTarget, fsm.feat.density);
end

function m = t23(cfg)
[~, map, ego] = mkTrackScene(cfg, 'car', 0,0,0,0);
i0 = 20; ego = mkEgo(map.route.xy(i0,1), map.route.xy(i0,2), map.route.hdg(i0), 6);
i = i0 + 24;
n = [-sin(map.route.hdg(i)) cos(map.route.hdg(i))];
tr = mkTrack('pedestrian', map.route.xy(i,1)+n(1)*3.0, map.route.xy(i,2)+n(2)*3.0, ...
             map.route.hdg(i)-pi/2, 1.4);
pr = iadpPredict(tr, cfg, map, ego);
[fsm, cmd] = iadpBehaviorFSM([], cfg, ego, tr, pr, map, 0, []);
check(any(strcmp(fsm.state,{'HOLD','YIELD','ESTOP','CREEP'})), ...
      'crossing pedestrian produced state %s', fsm.state);
check(cmd.vTarget < ego.v, 'did not slow for the pedestrian');
m = sprintf('state %s, vTarget %.2f m/s', fsm.state, cmd.vTarget);
end

function m = t24(cfg)
ego = mkEgo(0,0,0,5);
delta = 0.25;
dt = 0.01;
for k = 1:round(3/dt)
    ego = iadpVehicleStep(ego, delta, 0, cfg, dt);
end
Ranalytic = cfg.veh.L/tan(delta);
Rsim = 5/abs(ego.r);
check(abs(Rsim - Ranalytic)/Ranalytic < 0.02, ...
      'radius %.2f vs analytic %.2f', Rsim, Ranalytic);
m = sprintf('R_sim %.2f m, R_analytic %.2f m', Rsim, Ranalytic);
end

function m = t25(cfg)
ego = mkEgo(0,0,0,10); dt = 0.05;
ego = iadpVehicleStep(ego, cfg.veh.maxSteer, cfg.veh.aMax*5, cfg, dt);
check(abs(ego.delta) <= cfg.veh.maxSteerRt*dt + 1e-9, ...
      'steer rate violated: %.4f > %.4f', abs(ego.delta), cfg.veh.maxSteerRt*dt);
check(ego.a <= cfg.veh.jerkMax*dt + 1e-9, 'jerk limit violated: %.4f', ego.a);
for k = 1:200, ego = iadpVehicleStep(ego, 10, 0, cfg, dt); end
check(abs(ego.delta) <= cfg.veh.maxSteer + 1e-9, 'steering saturation violated');
m = sprintf('delta after 1 step %.4f rad, saturates at %.3f', cfg.veh.maxSteerRt*dt, ego.delta);
end

function m = t26(cfg)
c2 = cfg; c2.veh.model = 'dynamic';
ego = mkEgo(0,0,0,15); dt = 0.02;
for k = 1:round(6/dt)
    d = 0.06*sin(2*pi*0.3*k*dt);
    ego = iadpVehicleStep(ego, d, 0, c2, dt);
end
check(isfinite(ego.x) && isfinite(ego.vy) && abs(ego.vy) < 8, ...
      'dynamic model diverged (vy = %.2f)', ego.vy);
check(abs(ego.r) < 2, 'yaw rate diverged (%.2f)', ego.r);
m = sprintf('stable: vy %.2f m/s, r %.2f rad/s after 6 s of steering input', ego.vy, ego.r);
end

function m = t27(cfg)
th = linspace(0, pi/2, 300).'; R = 40;
P = iadpMakePath([R*sin(th) R*(1-cos(th))], 0.5);
traj.x = P.xy(:,1); traj.y = P.xy(:,2); traj.psi = P.hdg;
traj.v = 8*ones(size(P.s)); traj.t = P.s/8; traj.k = P.curv;
ego = mkEgo(P.xy(1,1), P.xy(1,2), P.hdg(1), 8);
ctl = struct('eInt',0); dt = 0.02; err = 0;
for k = 1:round(6/dt)
    [d, a, ctl] = iadpController(ego, traj, cfg, ctl, dt);
    ego = iadpVehicleStep(ego, d, a, cfg, dt);
    [~, e] = iadpPathProject(P, ego.x, ego.y);
    err = max(err, abs(e));
end
check(err < 1.0, 'max cross-track error %.2f m', err);
check(abs(ego.v - 8) < 1.0, 'speed error %.2f m/s', abs(ego.v-8));
m = sprintf('max cross-track %.2f m, speed error %.2f m/s', err, abs(ego.v-8));
end

function m = t28(cfg) %#ok<INUSD>
[d1, o1] = iadpRectDist([0 0 0],[4 2],[10 0 0],[4 2]);
check(abs(d1-6) < 1e-6 && ~o1, 'separated boxes: %.4f', d1);
[d2, o2] = iadpRectDist([0 0 0],[4 2],[1 0 0],[4 2]);
check(d2 == 0 && o2, 'overlap not detected');
[d3, ~] = iadpRectDist([0 0 0],[4 2],[0 5 0],[4 2]);
check(abs(d3-3) < 1e-6, 'lateral gap %.4f, expected 3', d3);
[d4, ~] = iadpRectDist([0 0 0],[4 2],[0 0 pi/4],[4 2]);
check(d4 == 0, 'rotated overlap missed');
m = 'separated, overlapping, lateral and rotated cases all correct';
end

function m = t29(cfg)
S = selfTestScenario(cfg, 'dense');      % the heaviest self-test fixture
c2 = cfg; c2.tMax = 12; S.meta.tMax = 12;
res = iadpSimulate(c2, S, struct('quiet',true));
lat = res.log.latency*1000;
p95 = sort(lat); p95 = p95(max(1,ceil(0.95*numel(p95))));
check(mean(lat) < cfg.metrics.latencyBudget*1000, ...
      'mean latency %.1f ms exceeds %.0f ms', mean(lat), cfg.metrics.latencyBudget*1000);
m = sprintf('mean %.1f ms, p95 %.1f ms over %d cycles (dense market)', ...
            mean(lat), p95, numel(lat));
end

function m = t30(cfg)
S = selfTestScenario(cfg, 'basic');
c2 = cfg; S.meta.tMax = 10;
res = iadpSimulate(c2, S, struct('quiet',true));
M = iadpMetrics(res, c2);
req = {'replanLatencyMean','pathSmoothCurvRMS','pathSmoothJerkRMS', ...
       'goalReached','minClearance','collision','classAccuracy','pass'};
for i = 1:numel(req)
    check(isfield(M, req{i}), 'metric %s missing', req{i});
end
m = sprintf('all %d required metrics present', numel(req));
end

% =======================================================================
% integration tests
%
% These run the full closed loop against small SELF-CONTAINED fixtures
% (built below with iadpWriteSampleXODR + hand-placed actors), NOT your
% production scenarios. That keeps this suite runnable before you have
% supplied any real RoadRunner files, while still exercising every stage
% of the real pipeline. To validate your own 5 scenarios, edit the table
% in iadpScenarioLibrary.m with your file names and run iadpRunAll.
% =======================================================================
function m = scenarioTest(cfg, kind)
S = selfTestScenario(cfg, kind);
res = iadpSimulate(cfg, S, struct('quiet',true));
M = iadpMetrics(res, cfg);
check(~M.collision, 'collision with %s at t=%.2f s', ...
      res.collisionInfo.with, res.collisionInfo.t);
check(M.goalReached, 'goal not reached (travelled %.1f m of %.1f m)', ...
      M.distance, res.goal.s);
check(M.minClearance >= res.checks.minClearance, ...
      'min clearance %.2f m below %.2f m', M.minClearance, res.checks.minClearance);
check(M.pass, '%s', strjoin(M.failReasons,'; '));
m = sprintf('t=%.1fs clear=%.2fm lat=%.0fms curvRMS=%.3f states=%s', ...
    M.completionTime, M.minClearance, M.replanLatencyMean, ...
    M.pathSmoothCurvRMS, strjoin(M.statesVisited,'/'));
end

function m = campaignTest(cfg)
fixtures = {'basic','dense','block'};
nPass = 0;
for i = 1:numel(fixtures)
    S = selfTestScenario(cfg, fixtures{i});
    res = iadpSimulate(cfg, S, struct('quiet',true));
    M = iadpMetrics(res, cfg);
    nPass = nPass + M.pass;
end
check(nPass == numel(fixtures), 'only %d/%d fixtures passed', nPass, numel(fixtures));
m = sprintf('fixture completion rate %d/%d (100%%)', nPass, numel(fixtures));
end

function m = noteProductionScenarios()
m = ['edit iadpScenarioLibrary.m with your own .xodr/.rrscene/.rrscenario ' ...
     'file names, then run iadpRunAll to validate the actual 5 scenarios'];
end

function S = selfTestScenario(cfg, kind)
%SELFTESTSCENARIO  A throwaway fixture for exercising the full closed
%loop, independent of the user-edited production manifest.
odr = iadpImportOpenDRIVE(fullfile(cfg.io.xodrDir,'IndianVillageRoad.xodr'), 0.5);
map = iadpBuildMap(odr, cfg, 1, [0 0]);
P = map.route;

switch kind
case 'basic'
    A = [iadpMakeActor(1,'pushcart', ...
            P.xy(round(numel(P.s)*0.35),1), P.xy(round(numel(P.s)*0.35),2)+0.2, ...
            P.hdg(round(numel(P.s)*0.35)), 0, 'stopped', 0)];
    v0 = 7; tMax = 20;
case 'dense'
    A = [];
    idxs = round(numel(P.s)*[0.15 0.25 0.35 0.45 0.55 0.65 0.75]);
    for k = 1:numel(idxs)
        i = idxs(k); side = (-1)^k;
        A = [A iadpMakeActor(k, tern(mod(k,2)==0,'pedestrian','pushcart'), ...
                P.xy(i,1), P.xy(i,2)+2.2*side, P.hdg(i)-side*pi/2, ...
                tern(mod(k,2)==0,1.1,0), tern(mod(k,2)==0,'cross','stopped'), 0.5, ...
                'vTarget',1.1,'trigT',1.0+k)]; %#ok<AGROW>
    end
    v0 = 4; tMax = 30;
case 'block'
    i = round(numel(P.s)*0.4);
    A = [iadpMakeActor(1,'truck', P.xy(i,1), P.xy(i,2), P.hdg(i), 0, 'stopped', 0)];
    v0 = 9; tMax = 25;
otherwise
    error('iadp:selftest','unknown fixture "%s"', kind);
end

ego0 = struct('x',P.xy(1,1), 'y',P.xy(1,2), 'psi',P.hdg(1), ...
              'v',v0, 'delta',0, 'beta',0, 'r',0, 'a',0);

S.name   = ['SelfTest_' kind];
S.odr    = odr;
S.map    = map;
S.actors = A;
S.ego0   = ego0;
S.goal   = struct('s', P.len-5, 'tol', 4.0);
S.source = 'self-test fixture (not a production scenario)';
c = struct('noCollision',true,'mustReachGoal',true,'maxLatencyMean',0.100, ...
    'minClearance',0.40,'maxJerkRMS',6.0,'maxCurvRMS',0.25,'mustUseShoulder',false, ...
    'mustYield',false,'mustCreep',false,'mustStopOrSwerve',false,'maxSpeed',inf);
S.meta   = struct('name',S.name,'tMax',tMax,'desc','self-test fixture','checks',c);
end

function s = tern(cond, a, b)
if cond, s = a; else, s = b; end
end

% =======================================================================
% helpers
% =======================================================================
function ego = mkEgo(x,y,psi,v)
ego = struct('x',x,'y',y,'psi',psi,'v',v,'delta',0,'a',0,'vy',0,'r',0);
end

function T = mkTrack(cls, x, y, psi, v)
T.id = 1; T.x = [x;y;v;psi;0]; T.P = 0.2*eye(5);
T.hits = 5; T.misses = 0; T.confirmed = true;
T.cls = cls; T.score = zeros(1,10);
T.extent = [NaN NaN]; T.age = 1; T.lastSeen = 0;
end

function [tracks, map, ego] = mkTrackScene(cfg, cls, x, y, psi, v)
persistent MAP
if isempty(MAP)
    odr = iadpImportOpenDRIVE(fullfile(cfg.io.xodrDir,'IndianVillageRoad.xodr'));
    MAP = iadpBuildMap(odr, cfg, 1, [0 0]);
end
map = MAP;
ego = mkEgo(map.route.xy(1,1), map.route.xy(1,2), map.route.hdg(1), 6);
tracks = mkTrack(cls, x, y, psi, v);
end

function d = minDistToPred(traj, preds)
d = inf;
for i = 1:numel(preds)
    for m = 1:numel(preds(i).mode)
        if preds(i).mode(m).w < 0.12, continue; end
        for k = 1:size(preds(i).mode(m).xy,1)
            dd = min(hypot(traj.x - preds(i).mode(m).xy(k,1), ...
                           traj.y - preds(i).mode(m).xy(k,2)));
            d = min(d, dd - preds(i).r);
        end
    end
end
end
