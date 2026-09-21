function spec = iadpScenarioLibrary(cfg, name)
%IADPSCENARIOLIBRARY  Manifest of the five mandatory Indian validation
%scenarios. This file names which files YOU must supply for each
%scenario and sets the ego's start/goal placement and pass criteria.
%It never invents road geometry or traffic - see iadpLoadRoadRunner.
%
%   spec = iadpScenarioLibrary(cfg)          % returns a cell of all names
%   spec = iadpScenarioLibrary(cfg, name)    % returns one scenario spec
%
%   Scenarios
%     1 VillageRoad        unmarked village road
%     2 UrbanIntersection  busy 4-way junction with NO signals
%     3 HighwayMerge       merging traffic from an on-ramp
%     4 MarketDense        dense market, mixed traffic
%     5 CattleCrossing     sudden animal crossing
%
%   For each scenario "Name", iadpLoadRoadRunner expects to find, under
%   cfg.io.xodrDir (default ./scenes) and cfg.rr.projectFolder (default
%   ./RRProject):
%
%     road network  - RRProject/Scenes/<spec.scene>.rrscene      (preferred, needs RoadRunner)
%                      OR  scenes/<spec.scene>.xodr               (OpenDRIVE export)
%     traffic       - RRProject/Scenarios/<spec.scenarioFile>.rrscenario  (preferred, live co-sim)
%                      OR  scenes/<spec.scenarioFile>_actors.csv  (id,class,x,y,heading_deg,speed,behavior,aggression)
%
%   Nothing here is auto-generated. If neither form is present the loader
%   raises an error naming the exact path it expected - it does not
%   substitute its own scene or traffic.
%
%   Each spec also carries:
%     .egoStartS .egoV0 .goalS .tMax .desc .checks (scenario pass criteria)
%   These are simulation parameters, not scene content, so they stay
%   here as editable defaults - change them to match your own scenario
%   file if your ego's start position or run length differs.

allNames = {'VillageRoad','UrbanIntersection','HighwayMerge', ...
            'MarketDense','CattleCrossing'};
if nargin < 2 || isempty(name)
    spec = allNames; return;
end

s = struct();
s.name         = name;
s.scenarioFile = name;
s.tMax         = cfg.tMax;
s.egoV0        = 6;
s.goalS        = inf;                 % clamped to route length by the loader
s.startXY      = [];
s.egoStartS    = 5;
s.checks       = defaultChecks();

switch name
%% ======================================================================
case 'VillageRoad'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 7;
    s.tMax      = 45;
    s.desc      = ['Narrow unmarked village road. Expected traffic: a ' ...
                   'parked pushcart, an oncoming bus, a pedestrian and ' ...
                   'an animal near the soft edge (define these in your ' ...
                   'RoadRunner Scenario). The ego must use the shoulder ' ...
                   'to pass and give way to the bus.'];
    s.checks.minClearance = 0.45;
    s.checks.mustUseShoulder = true;

%% ======================================================================
case 'UrbanIntersection'
    s.scene     = 'IndianUrbanIntersection';
    s.routeIdx  = [1 5 2];
    s.startXY   = [-78 0];
    s.egoV0     = 8;
    s.tMax      = 45;
    s.desc      = ['Unsignalised urban crossroads. Expected traffic: ' ...
                   'cross traffic that does not stop, an auto-rickshaw ' ...
                   'cutting in, pedestrians crossing informally. The ego ' ...
                   'must negotiate the junction by yielding, not by rule.'];
    s.checks.mustYield = true;

%% ======================================================================
case 'HighwayMerge'
    s.scene     = 'IndianHighwayMerge';
    s.routeIdx  = 1;
    s.startXY   = [-50 0];
    s.egoStartS = 10;
    s.egoV0     = 19;
    s.tMax      = 40;
    s.desc      = ['Highway with traffic merging from an on-ramp without ' ...
                   'signalling, plus a slow vehicle ahead in the running ' ...
                   'lane. The ego must either create a gap or overtake ' ...
                   'smoothly.'];
    s.checks.maxJerkRMS = 3.5;

%% ======================================================================
case 'MarketDense'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 4;
    s.tMax      = 60;
    s.desc      = ['Dense market street: pedestrians on the carriageway, ' ...
                   'parked pushcarts, weaving two-wheelers and a ' ...
                   'stopping auto-rickshaw. Expected behaviour is ' ...
                   'sustained low-speed creeping, not stopping.'];
    s.checks.maxSpeed      = 6.0;
    s.checks.mustCreep     = true;
    s.checks.minClearance  = 0.40;

%% ======================================================================
case 'CattleCrossing'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 11;
    s.tMax      = 40;
    s.desc      = ['Cattle emerge suddenly from the shoulder at cruising ' ...
                   'speed. The ego must detect, predict the crossing ' ...
                   'intent, brake and/or swerve, hold while the herd ' ...
                   'clears, then resume.'];
    s.checks.mustStopOrSwerve = true;
    s.checks.minClearance     = 0.50;

otherwise
    error('iadp:scenario','Unknown scenario "%s". Valid: %s', name, strjoin(allNames,', '));
end

s.allNames = allNames;
spec = s;
end

% =======================================================================
function c = defaultChecks()
c.noCollision        = true;
c.mustReachGoal      = true;
c.maxLatencyMean     = 0.100;   % [s]
c.minClearance       = 0.50;    % [m]
c.maxJerkRMS         = 5.0;     % [m/s^3]
c.maxCurvRMS         = 0.20;    % [1/m]
c.mustUseShoulder    = false;
c.mustYield          = false;
c.mustCreep          = false;
c.mustStopOrSwerve   = false;
c.maxSpeed           = inf;
end
