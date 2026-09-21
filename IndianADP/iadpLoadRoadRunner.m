function S = iadpLoadRoadRunner(cfg, scenarioName)
%IADPLOADROADRUNNER  Load a user-supplied scene (.rrscene/.xodr) and
%scenario (.rrscenario/.csv). Nothing is auto-generated: if the road
%network or the traffic cannot be resolved from YOUR files, this
%function throws an error naming the exact path it expected instead of
%substituting anything of its own.
%
%   S = iadpLoadRoadRunner(cfg, 'UrbanIntersection')
%
%   SCENE (road network) - first hit wins
%     1. RoadRunner API : open RRProject/Scenes/<scene>.rrscene and
%        export it to OpenDRIVE (requires RoadRunner + cfg.has.rrapi)
%     2. scenes/<scene>.xodr already present (e.g. your own RoadRunner
%        "Export to ASAM OpenDRIVE" output)
%     Neither present -> error.
%
%   SCENARIO (traffic) - first hit wins
%     1. RoadRunner Scenario live co-simulation: opens
%        RRProject/Scenarios/<scenarioFile>.rrscenario in RoadRunner,
%        starts the simulation, pauses it, and reads every actor's
%        initial pose and velocity (requires cfg.has.adt and a licensed
%        RoadRunner Scenario co-simulation interface).
%     2. scenes/<scenarioFile>_actors.csv - a plain interchange format
%        you can export/write yourself with columns:
%          id,class,x,y,heading_deg,speed,behavior,aggression
%        (behavior/aggression optional, default 'constant'/0.5). class
%        must be one of the labels in iadpClassParams.
%     Neither present -> error.
%
%   NOTE ON THE EGO ACTOR: if you also place an ego/AV actor inside your
%   RoadRunner Scenario, EXCLUDE it from cfg.rr.egoActorNames (cell of
%   actor names in your .rrscenario) so it is not double-counted as a
%   traffic obstacle - the ego pose and control are always computed by
%   this pipeline (iadpVehicleStep/iadpController), never read from RR.
%
%   Output
%     S.odr, S.map, S.actors, S.ego0, S.goal, S.name, S.source, S.meta

spec = iadpScenarioLibrary(cfg, scenarioName);
S.name = spec.name;
S.meta = spec;

%% ---------------- scene (road network) ---------------------------------
xodr    = fullfile(cfg.io.xodrDir, [spec.scene '.xodr']);
rrscene = fullfile(cfg.rr.projectFolder, 'Scenes', [spec.scene '.rrscene']);
src = '';

if cfg.rr.enable && cfg.has.rrapi && exist(rrscene,'file')
    try
        xodr = exportFromRoadRunner(cfg, spec.scene, xodr);
        src  = 'RoadRunner API export';
    catch ME
        warning('iadp:rr', ...
            'RoadRunner scene export failed for "%s" (%s). Falling back to an existing .xodr, if any.', ...
            spec.scene, ME.message);
    end
end

if isempty(src)
    if exist(xodr,'file')
        src = 'OpenDRIVE file you supplied';
    else
        error('iadp:scene:notFound', [ ...
            'No road network found for scenario "%s".\n' ...
            'Expected one of:\n' ...
            '  %s   (a RoadRunner .rrscene, opened via the RoadRunner API)\n' ...
            '  %s   (an exported OpenDRIVE file)\n' ...
            'Export your RoadRunner scene to OpenDRIVE (File > Export > ASAM OpenDRIVE)\n' ...
            'and place it at the second path above, or point cfg.rr.projectFolder /\n' ...
            'cfg.rr.installFolder at a working RoadRunner installation so the API\n' ...
            'export can run automatically. This pipeline does not generate its own\n' ...
            'road geometry.'], spec.name, rrscene, xodr);
    end
end

S.odr    = iadpImportOpenDRIVE(xodr, 0.5);
S.map    = iadpBuildMap(S.odr, cfg, spec.routeIdx, spec.startXY);
S.source = sprintf('%s (%s)', src, xodr);

%% ---------------- ego ---------------------------------------------------
P  = S.map.route;
i0 = find(P.s >= spec.egoStartS, 1, 'first');
if isempty(i0), i0 = 1; end
S.ego0 = struct('x',P.xy(i0,1), 'y',P.xy(i0,2), 'psi',P.hdg(i0), ...
                'v',spec.egoV0, 'delta',0, 'beta',0, 'r',0, 'a',0);
S.goal = struct('s', min(P.len - 3, spec.goalS), 'tol', 4.0);

%% ---------------- actors (traffic) --------------------------------------
rrscenario = fullfile(cfg.rr.projectFolder, 'Scenarios', [spec.scenarioFile '.rrscenario']);
csv        = fullfile(cfg.io.xodrDir, [spec.scenarioFile '_actors.csv']);
actors = [];
actorSrc = '';

if cfg.rr.enable && cfg.has.adt && exist(rrscenario,'file')
    try
        actors = actorsFromRRScenario(cfg, spec, rrscenario);
        actorSrc = 'RoadRunner Scenario live co-simulation';
    catch ME
        warning('iadp:rr', ...
            'RoadRunner Scenario read failed for "%s" (%s). Falling back to an actors CSV, if any.', ...
            spec.scenarioFile, ME.message);
    end
end

if isempty(actorSrc)
    if exist(csv,'file')
        actors   = actorsFromCSV(csv);
        actorSrc = 'actors CSV you supplied';
    else
        error('iadp:scenario:notFound', [ ...
            'No traffic definition found for scenario "%s".\n' ...
            'Expected one of:\n' ...
            '  %s   (a RoadRunner .rrscenario, read via RoadRunner Scenario co-simulation)\n' ...
            '  %s   (a plain CSV: id,class,x,y,heading_deg,speed,behavior,aggression)\n' ...
            'Define the traffic in RoadRunner Scenario and either keep the project\n' ...
            'linked (cfg.rr.projectFolder) so it can be read live, or write out a CSV\n' ...
            'at the second path above. This pipeline does not invent its own traffic.'], ...
            spec.name, rrscenario, csv);
    end
end
S.actors = actors;

if cfg.io.verbose
    fprintf('[iadp] scenario "%s": scene = %s | traffic = %s | %d actors | route %.0f m\n', ...
        S.name, S.source, actorSrc, numel(S.actors), P.len);
end
end

% =======================================================================
function xodr = exportFromRoadRunner(cfg, sceneName, xodr)
% Uses the MATLAB<->RoadRunner API. Requires RoadRunner + a valid project.
rrApp = roadrunner(cfg.rr.projectFolder, 'InstallationFolder', cfg.rr.installFolder);
openScene(rrApp, [sceneName '.rrscene']);
exportScene(rrApp, xodr, 'OpenDRIVE');
close(rrApp);
end

% =======================================================================
function actors = actorsFromRRScenario(cfg, spec, rrscenarioPath) %#ok<INUSD>
% Pulls the initial actor set out of a .rrscenario through the
% RoadRunner Scenario co-simulation interface. Any actor whose name
% matches cfg.rr.egoActorNames is treated as the ego and skipped, since
% the ego is always simulated by this pipeline, never read from RR.
actors = [];
rrApp = roadrunner(cfg.rr.projectFolder, 'InstallationFolder', cfg.rr.installFolder);
openScenario(rrApp, spec.scenarioFile);
sim = createSimulation(rrApp);
set(sim,'SimulationCommand','Start'); pause(0.5);
set(sim,'SimulationCommand','Pause');
w = get(sim,'Actors');
egoNames = {};
if isfield(cfg.rr,'egoActorNames'), egoNames = cfg.rr.egoActorNames; end
kId = 0;
for k = 1:numel(w)
    a = w(k);
    if isfield(a,'Name') && any(strcmpi(char(a.Name), egoNames))
        continue   % this actor is the ego - skip, our own model drives it
    end
    kId = kId + 1;
    actors = [actors iadpMakeActor(kId, mapRRClass(a.ActorType), ...
        a.Pose(1,4), a.Pose(2,4), atan2(a.Pose(2,1),a.Pose(1,1)), ...
        norm(a.Velocity(1:2)), 'rrTrack', 0.5)]; %#ok<AGROW>
end
set(sim,'SimulationCommand','Stop');
close(rrApp);
if isempty(actors)
    error('iadp:rr:noActors','RoadRunner Scenario "%s" opened but returned no traffic actors.', spec.scenarioFile);
end
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
if isempty(actors)
    error('iadp:scenario:emptyCSV','%s exists but contains no rows.', csv);
end
end
