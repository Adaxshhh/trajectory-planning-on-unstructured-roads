function [res, M] = iadpOpenInRoadRunner(name, cfg)
%IADPOPENINROADRUNNER  One command: detect-and-dodge in RoadRunner.
%
%   [res, M] = iadpOpenInRoadRunner('CattleCrossing')
%   [res, M] = iadpOpenInRoadRunner('VillageRoad', cfg)
%
%   1. Generates the road network and traffic for the scenario.
%   2. Runs the full closed-loop pipeline in MATLAB (sensors -> fusion ->
%      prediction -> risk grid -> FSM -> planner -> A* -> controller ->
%      vehicle model) and prints detailed statistics.
%   3. Exports ego + traffic trajectories as an OpenSCENARIO file.
%   4. Opens RoadRunner, builds <scene>.rrscene from the generated
%      OpenDRIVE (newScene/importScene/saveScene), builds
%      <name>.rrscenario from the trajectories (newScenario/
%      importScenario/saveScenario) and plays it, so you watch the ego
%      detect and dodge the traffic inside RoadRunner.
%   Works on MATLAB R2022a+ with RoadRunner + RoadRunner Scenario: it
%   needs no behaviour asset and no authoring API. If RoadRunner is not
%   available, steps 1-3 still run and the files are left in ./results.

if nargin < 2, cfg = iadpSetup('io.video', false); end
cfg.io.video = false;
iadpWriteSampleXODR(cfg);
S   = iadpLoadRoadRunner(cfg, name);
res = iadpSimulate(cfg, S);
M   = iadpMetrics(res, cfg);
iadpStatsReport(res, M, cfg);
try, iadpVisualize(res, cfg, 'plots'); catch, end

if ~cfg.has.rrapi
    error('iadp:rr','RoadRunner not detected. Set cfg.rr.installFolder, e.g. iadpSetup(''rr.installFolder'',''C:\\Program Files\\RoadRunner R2025a'').');
end
iadpPlayInRoadRunner(res, S.meta, cfg);
end
