function S = iadpLoadRoadRunner(cfg, scenarioName)
%IADPLOADROADRUNNER  Load a scene (.rrscene/.xodr) and a scenario (.rrscenario/.csv).
%
%   S = iadpLoadRoadRunner(cfg, 'UrbanIntersection')
%
%   Resolution order (first hit wins), so the same code works whether or not
%   RoadRunner is installed:
%
%   SCENE (road network)
%     1. RoadRunner API : open <scene>.rrscene and Export to OpenDRIVE
%     2. <scene>.xodr already present in cfg.io.xodrDir   <-- RoadRunner export
%     3. built-in generator iadpWriteSampleXODR (equivalent geometry)
%
%   SCENARIO (actors)
%     1. RoadRunner Scenario API (roadrunnerScenarioSimulation) actor list
%     2. <scenario>_actors.csv in cfg.io.xodrDir
%        columns: id,class,x,y,heading_deg,speed,behavior,aggression
%     3. built-in iadpScenarioLibrary definition
%
%   Output
%     S.odr, S.map, S.actors, S.ego0, S.goal, S.name, S.source, S.meta

spec = iadpScenarioLibrary(cfg, scenarioName);
S.name = spec.name;
S.meta = spec;

%% ---------------- scene ------------------------------------------------
xodr = fullfile(cfg.io.xodrDir, [spec.scene '.xodr']);
src  = '';

if cfg.rr.enable && cfg.has.rrapi && exist(fullfile(cfg.rr.projectFolder,'Scenes',[spec.scene '.rrscene']),'file')
    try
        xodr = exportFromRoadRunner(cfg, spec.scene, xodr);
        src  = 'RoadRunner API export';
    catch ME
        warning('iadp:rr','RoadRunner export failed (%s) - falling back.', ME.message);
    end
end

if isempty(src)
    if ~exist(xodr,'file')
        iadpWriteSampleXODR(cfg);
    end
    if exist(xodr,'file')
        src = 'OpenDRIVE file';
    else
        error('iadp:scene','Cannot resolve scene %s', spec.scene);
    end
end

S.odr    = iadpImportOpenDRIVE(xodr, 0.5);
S.map    = iadpBuildMap(S.odr, cfg, spec.routeIdx, spec.startXY);
S.source = sprintf('%s (%s)', src, xodr);

%% ---------------- ego --------------------------------------------------
P  = S.map.route;
i0 = find(P.s >= spec.egoStartS, 1, 'first');
if isempty(i0), i0 = 1; end
S.ego0 = struct('x',P.xy(i0,1), 'y',P.xy(i0,2), 'psi',P.hdg(i0), ...
                'v',spec.egoV0, 'delta',0, 'beta',0, 'r',0, 'a',0);
S.goal = struct('s', min(P.len - 3, spec.goalS), 'tol', 4.0);

%% ---------------- actors -----------------------------------------------
actors = [];
if cfg.rr.enable && cfg.has.adt && exist('roadrunnerScenarioSimulation','file')
    try
        actors = actorsFromRRScenario(cfg, spec);
    catch ME
        warning('iadp:rr','RoadRunner Scenario read failed (%s).', ME.message);
    end
end
if isempty(actors)
    csv = fullfile(cfg.io.xodrDir, [spec.scenarioFile '_actors.csv']);
    if exist(csv,'file'), actors = actorsFromCSV(csv); end
end
if isempty(actors)
    actors = spec.actorFcn(S.map, cfg);   % built-in definition
end
S.actors = actors;

if cfg.io.verbose
    fprintf('[iadp] scenario "%s": %s | %d actors | route %.0f m\n', ...
        S.name, S.source, numel(S.actors), P.len);
end
end

% =======================================================================
function xodr = exportFromRoadRunner(cfg, sceneName, xodr)
% Uses the MATLAB<->RoadRunner API. Requires RoadRunner + a valid project.
rrApp = roadrunner(cfg.rr.projectFolder, 'InstallationFolder', cfg.rr.installFolder);
openScene(rrApp, [sceneName '.rrscene']);
opts = roadrunnerExportOptions('OpenDRIVE');    %#ok<NASGU>  (version dependent)
exportScene(rrApp, xodr, 'OpenDRIVE');
close(rrApp);
end

% =======================================================================
function actors = actorsFromRRScenario(cfg, spec)
% Pulls the initial actor set out of a .rrscenario through the
% RoadRunner Scenario co-simulation interface.
actors = [];
rrApp = roadrunner(cfg.rr.projectFolder, 'InstallationFolder', cfg.rr.installFolder);
openScenario(rrApp, spec.scenarioFile);
sim = createSimulation(rrApp);
set(sim,'SimulationCommand','Start'); pause(0.5);
set(sim,'SimulationCommand','Pause');
w = get(sim,'Actors');
for k = 1:numel(w)
    a = w(k);
    actors = [actors iadpMakeActor(double(k), mapRRClass(a.ActorType), ...
        a.Pose(1,4), a.Pose(2,4), atan2(a.Pose(2,1),a.Pose(1,1)), ...
        norm(a.Velocity(1:2)), 'rrTrack', 0.5)]; %#ok<AGROW>
end
set(sim,'SimulationCommand','Stop');
close(rrApp);
end

function c = mapRRClass(t)
t = lower(char(t));
if contains(t,'pedestr'),        c = 'pedestrian';
elseif contains(t,'bicycle'),    c = 'bicycle';
elseif contains(t,'motor') || contains(t,'bike'), c = 'twowheeler';
elseif contains(t,'truck'),      c = 'truck';
elseif contains(t,'bus'),        c = 'bus';
elseif contains(t,'animal') || contains(t,'cow'), c = 'animal';
else,                            c = 'car';
end
end

% =======================================================================
function actors = actorsFromCSV(csv)
T = readtable(csv);
actors = [];
for k = 1:height(T)
    beh = 'constant';
    if ismember('behavior', T.Properties.VariableNames), beh = char(T.behavior(k)); end
    ag  = 0.5;
    if ismember('aggression', T.Properties.VariableNames), ag = T.aggression(k); end
    actors = [actors iadpMakeActor(T.id(k), char(T.class(k)), T.x(k), T.y(k), ...
        deg2rad(T.heading_deg(k)), T.speed(k), beh, ag)]; %#ok<AGROW>
end
end
