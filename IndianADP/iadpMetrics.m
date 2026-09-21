function M = iadpMetrics(res, cfg)
%IADPMETRICS  Quantitative evaluation of one simulation run.
%
%   M = iadpMetrics(res, cfg)
%
%   Reported (the three required by the brief are marked *):
%     * replanLatencyMean/P95/Max  [ms]  full perception->planning cycle
%     * pathSmoothCurvRMS          [1/m] RMS path curvature
%       pathSmoothJerkRMS          [m/s^3]
%     * completed                  goal reached without collision
%       minClearance               [m]  true rectangle-to-rectangle gap
%       minTTC                     [s]
%       collision                  logical
%       meanSpeed / maxSpeed       [m/s]
%       maxLatAcc                  [m/s^2]
%       replans, astarCalls, astarSuccessRate
%       classAccuracy              fused classification vs ground truth
%       shoulderUse                fraction of time on the soft edge
%       pass                       all scenario checks satisfied
%       failReasons                cell array of what failed

L = res.log;
n = numel(L.t);
dt = cfg.dt;

M.name = res.name;

%% ---- latency ----------------------------------------------------------
lat = L.latency(:)*1000;
M.replanCount        = numel(lat);
M.replanLatencyMean  = mean(lat);
M.replanLatencyP95   = prctileLocal(lat, 95);
M.replanLatencyMax   = max(lat);
M.replanRateHz       = M.replanCount/max(res.tEnd,eps);

%% ---- smoothness -------------------------------------------------------
M.pathSmoothCurvRMS = sqrt(mean(L.curv.^2));
acc  = gradient(L.v, dt);
jerk = gradient(acc, dt);
M.pathSmoothJerkRMS = sqrt(mean(jerk.^2));
M.maxLatAcc = max(abs(L.v.^2 .* L.curv));
% lateral smoothness of the driven line
dpsi = gradient(unwrap(L.psi), dt);
M.yawRateRMS = sqrt(mean(dpsi.^2));

%% ---- safety -----------------------------------------------------------
M.collision    = res.collision;
M.minClearance = min(L.minClear);
M.minTTC       = min(L.ttc);
M.nearMisses   = sum(L.minClear < cfg.safe.minClear)*dt;

%% ---- progress ---------------------------------------------------------
M.goalReached  = res.goalReached;
M.completionTime = res.goalTime;
M.distance     = L.s(end) - L.s(1);
M.meanSpeed    = mean(L.v);
M.maxSpeed     = max(L.v);
M.stuck        = res.stuck;
M.shoulderUse  = mean(L.onShoulder);

%% ---- planner health ---------------------------------------------------
M.planFailures = L.planFail;
M.astarCalls   = L.astarCalls;
if L.astarCalls > 0
    M.astarSuccessRate = L.astarSuccess/L.astarCalls;
    M.astarTimeMean    = mean(L.astar)*1000;
else
    M.astarSuccessRate = NaN; M.astarTimeMean = NaN;
end

%% ---- perception -------------------------------------------------------
if L.classN > 0
    M.classAccuracy = L.classHit/L.classN;
else
    M.classAccuracy = NaN;
end
M.meanTracks = mean(L.nTracks);

%% ---- behaviour coverage ----------------------------------------------
states = unique(L.state);
M.statesVisited = states(:).';
M.usedYield  = any(strcmp(L.state,'YIELD'));
M.usedCreep  = any(strcmp(L.state,'CREEP'));
M.usedHold   = any(strcmp(L.state,'HOLD'));
M.usedEStop  = any(strcmp(L.state,'ESTOP'));
M.usedObst   = any(strcmp(L.state,'OBSTACLE'));

%% ---- scenario checks --------------------------------------------------
c = res.checks;
fails = {};
if c.noCollision   && M.collision,      fails{end+1} = 'collision occurred'; end
if c.mustReachGoal && ~M.goalReached,   fails{end+1} = 'goal not reached'; end
if M.replanLatencyMean > c.maxLatencyMean*1000
    fails{end+1} = sprintf('mean replan latency %.0f ms > %.0f ms', ...
                            M.replanLatencyMean, c.maxLatencyMean*1000);
end
if M.minClearance < c.minClearance
    fails{end+1} = sprintf('min clearance %.2f m < %.2f m', M.minClearance, c.minClearance);
end
if M.pathSmoothJerkRMS > c.maxJerkRMS
    fails{end+1} = sprintf('jerk RMS %.2f > %.2f', M.pathSmoothJerkRMS, c.maxJerkRMS);
end
if M.pathSmoothCurvRMS > c.maxCurvRMS
    fails{end+1} = sprintf('curvature RMS %.3f > %.3f', M.pathSmoothCurvRMS, c.maxCurvRMS);
end
if M.maxSpeed > c.maxSpeed
    fails{end+1} = sprintf('max speed %.1f > %.1f m/s', M.maxSpeed, c.maxSpeed);
end
if c.mustUseShoulder && M.shoulderUse <= 0
    fails{end+1} = 'never used the shoulder to pass the blockage';
end
if c.mustYield && ~(M.usedYield || M.usedHold || M.usedEStop)
    fails{end+1} = 'never yielded at the unsignalised junction';
end
if c.mustCreep && ~M.usedCreep
    fails{end+1} = 'never entered CREEP in the dense market';
end
if c.mustStopOrSwerve && ~(M.usedHold || M.usedEStop || M.usedObst)
    fails{end+1} = 'did not react to the cattle crossing';
end
if M.stuck
    fails{end+1} = 'vehicle became permanently stuck';
end

M.failReasons = fails;
M.pass = isempty(fails);
end

% =======================================================================
function v = prctileLocal(x, p)
x = sort(x(:));
if isempty(x), v = NaN; return; end
i = max(1, min(numel(x), ceil(p/100*numel(x))));
v = x(i);
end
