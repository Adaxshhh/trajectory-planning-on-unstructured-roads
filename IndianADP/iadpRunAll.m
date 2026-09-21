function R = iadpRunAll(varargin)
%IADPRUNALL  Run the complete validation campaign on YOUR RoadRunner data.
%
%   R = iadpRunAll()                      % all 5 scenarios, video on
%   R = iadpRunAll('video',false)         % faster, no AVI export
%   R = iadpRunAll('scenarios',{'CattleCrossing'})
%   R = iadpRunAll('model','dynamic')     % use the dynamic bicycle model
%
%   This function never generates its own scene or traffic. For every
%   scenario it runs, iadpLoadRoadRunner expects a real RoadRunner scene
%   (.rrscene, or its exported .xodr) and a real scenario (.rrscenario,
%   or an actors CSV) that you have already placed under cfg.rr.projectFolder
%   / cfg.io.xodrDir - see the file index in Contents.m and the header of
%   iadpScenarioLibrary.m for the exact expected paths and names. If a
%   file is missing you will get a clear error naming the path it looked
%   for, not a substitute scene.
%
%   Produces, in ./results :
%     <scenario>.avi           demonstration video
%     <scenario>_metrics.png   speed / latency / clearance / state plots
%     iadp_results.mat         every log and metric
%     iadp_report.txt          the technical summary table
%
%   This is the single command a reviewer needs to run, once your
%   RoadRunner assets are in place.

p = inputParser;
p.addParameter('video', true);
p.addParameter('plots', true);
p.addParameter('scenarios', {});
p.addParameter('model', 'kinematic');
p.addParameter('seed', 7);
p.parse(varargin{:});
o = p.Results;

cfg = iadpSetup('io.video', o.video, 'veh.model', o.model, 'rngSeed', o.seed);

names = o.scenarios;
if isempty(names), names = iadpScenarioLibrary(cfg); end

preflightCheck(cfg, names);

R = struct('res',{},'M',{});
fprintf('\n================ IADP validation campaign ================\n');

for i = 1:numel(names)
    fprintf('\n[%d/%d] %s\n', i, numel(names), names{i});
    S   = iadpLoadRoadRunner(cfg, names{i});
    res = iadpSimulate(cfg, S);
    M   = iadpMetrics(res, cfg);

    if o.video || o.plots
        what = 'all';
        if ~o.video, what = 'plots'; end
        if ~o.plots, what = 'video'; end
        try
            iadpVisualize(res, cfg, what);
        catch ME
            warning('iadp:viz','visualisation failed: %s', ME.message);
        end
    end

    R(end+1).res = res; %#ok<AGROW>
    R(end).M = M;
    printOne(M);
end

printTable(R);
printCoverage(R, cfg);

save(fullfile(cfg.io.outDir,'iadp_results.mat'), 'R', 'cfg', '-v7.3');
writeReport(R, cfg);
fprintf('\nArtefacts written to %s\n', cfg.io.outDir);
end

% =======================================================================
function preflightCheck(cfg, names)
% Reports every missing scene/scenario file up front, across all
% requested scenarios, instead of failing one at a time mid-campaign.
missing = {};
for i = 1:numel(names)
    spec = iadpScenarioLibrary(cfg, names{i});
    xodr    = fullfile(cfg.io.xodrDir, [spec.scene '.xodr']);
    rrscene = fullfile(cfg.rr.projectFolder, 'Scenes', [spec.scene '.rrscene']);
    if ~exist(xodr,'file') && ~exist(rrscene,'file')
        missing{end+1} = sprintf('  %-18s scene:   %s', names{i}, xodr); %#ok<AGROW>
    end
    csv        = fullfile(cfg.io.xodrDir, [spec.scenarioFile '_actors.csv']);
    rrscenario = fullfile(cfg.rr.projectFolder, 'Scenarios', [spec.scenarioFile '.rrscenario']);
    if ~exist(csv,'file') && ~exist(rrscenario,'file')
        missing{end+1} = sprintf('  %-18s traffic: %s', names{i}, csv); %#ok<AGROW>
    end
end
if ~isempty(missing)
    error('iadp:preflight', [ ...
        'The following RoadRunner scene/scenario files are missing and ' ...
        'must be supplied before running iadpRunAll (nothing is auto-generated):\n%s'], ...
        strjoin(missing, '\n'));
end
end

% =======================================================================
function printOne(M)
fprintf('    goal %d | collision %d | minClear %.2f m | minTTC %.2f s\n', ...
    M.goalReached, M.collision, M.minClearance, M.minTTC);
fprintf('    latency  mean %.1f ms  p95 %.1f ms  max %.1f ms  (%d replans)\n', ...
    M.replanLatencyMean, M.replanLatencyP95, M.replanLatencyMax, M.replanCount);
fprintf('    smoothness curvRMS %.4f 1/m  jerkRMS %.2f m/s^3  maxLatAcc %.2f m/s^2\n', ...
    M.pathSmoothCurvRMS, M.pathSmoothJerkRMS, M.maxLatAcc);
fprintf('    classification %.0f%% | states: %s\n', 100*M.classAccuracy, strjoin(M.statesVisited,','));
if M.pass
    fprintf('    RESULT: PASS\n');
else
    fprintf('    RESULT: FAIL -> %s\n', strjoin(M.failReasons,'; '));
end
end

% =======================================================================
function printTable(R)
fprintf('\n================ summary ================\n');
fprintf('%-20s %6s %6s %9s %9s %9s %9s %6s\n', ...
    'scenario','goal','coll','lat_ms','curvRMS','jerkRMS','clear_m','pass');
for i = 1:numel(R)
    M = R(i).M;
    fprintf('%-20s %6d %6d %9.1f %9.4f %9.2f %9.2f %6d\n', ...
        M.name, M.goalReached, M.collision, M.replanLatencyMean, ...
        M.pathSmoothCurvRMS, M.pathSmoothJerkRMS, M.minClearance, M.pass);
end
np = sum(arrayfun(@(r) r.M.pass, R));
fprintf('-----------------------------------------\n');
fprintf('scenario completion rate : %d/%d (%.0f%%)\n', np, numel(R), 100*np/numel(R));
end

% =======================================================================
function printCoverage(R, cfg)
req = {
 'Multi-sensor perception (camera + LiDAR + radar)', 'iadpSensors.m'
 'Sensor fusion / multi-object tracking',            'iadpFusion.m (EKF-CTRV)'
 'Diverse Indian class recognition',                 'iadpClassParams / fusion classify'
 'Short-term motion prediction (non-lane-based)',    'iadpPredict.m (4 modes)'
 'Collision-free path generation',                   'iadpLocalPlanner.m'
 'Real-time replanning',                             'iadpSimulate.m @10 Hz'
 'Unstructured / missing-lane planning',             'iadpHybridAStar.m + risk grid'
 'Decision logic (Stateflow-equivalent)',            'iadpBehaviorFSM.m'
 'Vehicle dynamics (bicycle model)',                 'iadpVehicleStep.m'
 'RoadRunner scene + .xodr ingestion',               'iadpLoadRoadRunner / iadpImportOpenDRIVE'
 'Five Indian scenarios',                            'iadpScenarioLibrary.m'
 'Metrics: latency, smoothness, completion',         'iadpMetrics.m'
 'Closed-loop validation',                           'iadpRunAll.m'
 'Demonstration video',                              'iadpVisualize.m'
};
fprintf('\n================ requirement coverage ================\n');
for i = 1:size(req,1)
    fprintf('  [x] %-52s %s\n', req{i,1}, req{i,2});
end
fprintf('  Toolboxes used when present: ADT=%d Nav=%d DL=%d Stateflow=%d RR-API=%d\n', ...
    cfg.has.adt, cfg.has.nav, cfg.has.dl, cfg.has.stateflow, cfg.has.rrapi);
end

% =======================================================================
function writeReport(R, cfg)
f = fullfile(cfg.io.outDir,'iadp_report.txt');
fid = fopen(f,'w');
fprintf(fid,'Adaptive Path Planning for Unstructured Indian Roads\n');
fprintf(fid,'Generated %s\n\n', datestr(now));
fprintf(fid,'%-20s %6s %6s %9s %9s %9s %9s %8s %6s\n', ...
    'scenario','goal','coll','lat_ms','p95_ms','curvRMS','jerkRMS','clear_m','pass');
for i = 1:numel(R)
    M = R(i).M;
    fprintf(fid,'%-20s %6d %6d %9.1f %9.1f %9.4f %9.2f %8.2f %6d\n', ...
        M.name, M.goalReached, M.collision, M.replanLatencyMean, ...
        M.replanLatencyP95, M.pathSmoothCurvRMS, M.pathSmoothJerkRMS, ...
        M.minClearance, M.pass);
end
fprintf(fid,'\nPer-scenario detail\n');
for i = 1:numel(R)
    M = R(i).M; res = R(i).res;
    fprintf(fid,'\n--- %s ---\n%s\n', M.name, res.desc);
    fprintf(fid,'  completion time      : %.2f s\n', M.completionTime);
    fprintf(fid,'  mean / max speed     : %.2f / %.2f m/s\n', M.meanSpeed, M.maxSpeed);
    fprintf(fid,'  min clearance / TTC  : %.2f m / %.2f s\n', M.minClearance, M.minTTC);
    fprintf(fid,'  replans              : %d at %.1f Hz\n', M.replanCount, M.replanRateHz);
    fprintf(fid,'  hybrid A* calls      : %d (success %.0f%%)\n', M.astarCalls, 100*M.astarSuccessRate);
    fprintf(fid,'  classification acc.  : %.0f%%\n', 100*M.classAccuracy);
    fprintf(fid,'  shoulder usage       : %.0f%% of time\n', 100*M.shoulderUse);
    fprintf(fid,'  behaviour states     : %s\n', strjoin(M.statesVisited,', '));
    if ~M.pass
        fprintf(fid,'  FAILURES             : %s\n', strjoin(M.failReasons,'; '));
    end
    fprintf(fid,'  state transitions:\n');
    for j = 1:numel(res.fsmLog)
        fprintf(fid,'    %s\n', res.fsmLog{j});
    end
end
fclose(fid);
end
