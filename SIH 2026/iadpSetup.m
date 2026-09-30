function cfg = iadpSetup(varargin)
%IADPSETUP  Configuration + path setup for the Indian Adaptive Driving Planner.
%
%   cfg = iadpSetup()                 % default configuration
%   cfg = iadpSetup('dt',0.05,...)    % override any field (name/value)
%
%   Everything in the pipeline reads its parameters from this single struct.
%   Run this once per session:  cfg = iadpSetup;
%
%   Toolboxes are OPTIONAL. The code detects what is installed and uses a
%   built-in fallback implementation otherwise, so the full 5-scenario
%   validation runs on base MATLAB.

%% ---- make sure this folder is on the path -------------------------------
here = fileparts(mfilename('fullpath'));
if ~contains(path, here)
    addpath(here);
end

%% ---- simulation --------------------------------------------------------
cfg.dt            = 0.05;    % [s] integration step (20 Hz vehicle)
cfg.planRate      = 10;      % [Hz] local re-plan rate
cfg.tMax          = 60;      % [s] hard cap per scenario
cfg.rngSeed       = 7;       % reproducibility

%% ---- ego vehicle (Simulink bicycle-model equivalent) -------------------
cfg.veh.L         = 2.70;    % [m] wheelbase
cfg.veh.lf        = 1.25;    % [m] CG to front axle (dynamic model)
cfg.veh.lr        = 1.45;    % [m] CG to rear axle
cfg.veh.m         = 1600;    % [kg]
cfg.veh.Iz        = 2800;    % [kg m^2]
cfg.veh.Cf        = 80000;   % [N/rad] front cornering stiffness
cfg.veh.Cr        = 90000;   % [N/rad] rear cornering stiffness
cfg.veh.width     = 1.80;    % [m]
cfg.veh.length    = 4.40;    % [m]
cfg.veh.rearOH    = 0.90;    % [m] rear overhang (rear-axle reference frame)
cfg.veh.maxSteer  = 0.58;    % [rad]
cfg.veh.maxSteerRt= 0.90;    % [rad/s]
cfg.veh.aMax      = 2.20;    % [m/s^2]
cfg.veh.aMin      = -5.50;   % [m/s^2] emergency braking
cfg.veh.jerkMax   = 4.00;    % [m/s^3]
cfg.veh.latAccMax = 2.50;    % [m/s^2] comfort limit -> curvature speed cap
cfg.veh.model     = 'kinematic';   % 'kinematic' | 'dynamic'

%% ---- controller --------------------------------------------------------
cfg.ctrl.LdMin    = 3.0;     % [m] pure-pursuit lookahead floor
cfg.ctrl.LdGain   = 0.9;     % [s] lookahead = LdGain*v
cfg.ctrl.LdMax    = 14.0;
cfg.ctrl.Kp       = 1.6;     % speed loop
cfg.ctrl.Ki       = 0.25;
cfg.ctrl.iMax     = 3.0;

%% ---- sensors -----------------------------------------------------------
% Camera: good classification, poor range accuracy
cfg.sensor.cam.range    = 70;   cfg.sensor.cam.fov = deg2rad(120);
cfg.sensor.cam.sigRange = 0.90; cfg.sensor.cam.sigAz = deg2rad(0.6);
cfg.sensor.cam.pd       = 0.93; cfg.sensor.cam.classAcc = 0.90;
cfg.sensor.cam.mountXY  = [1.5 0];
% LiDAR: accurate geometry + extent, no class
cfg.sensor.lidar.range    = 90;  cfg.sensor.lidar.fov = 2*pi;
cfg.sensor.lidar.sigRange = 0.06; cfg.sensor.lidar.sigAz = deg2rad(0.2);
cfg.sensor.lidar.pd       = 0.97; cfg.sensor.lidar.sigExtent = 0.15;
cfg.sensor.lidar.mountXY  = [1.0 0];
% Radar: long range + range-rate, coarse lateral, robust to dust/rain
cfg.sensor.radar.range    = 140; cfg.sensor.radar.fov = deg2rad(90);
cfg.sensor.radar.sigRange = 0.25; cfg.sensor.radar.sigAz = deg2rad(2.0);
cfg.sensor.radar.sigRate  = 0.12; cfg.sensor.radar.pd = 0.90;
cfg.sensor.radar.mountXY  = [2.2 0];
cfg.sensor.clutterRate    = 0.6;   % expected false alarms per scan
cfg.sensor.occlusionOn    = true;

%% ---- tracking / fusion (EKF-CTRV, class-adaptive) ----------------------
cfg.track.gate        = 9.21;   % chi^2, 2 dof, 99%
cfg.track.confirmHits = 3;
cfg.track.maxMisses   = 8;
cfg.track.R_cam       = diag([1.20 1.20].^2);
cfg.track.R_lidar     = diag([0.12 0.12].^2);
cfg.track.R_radar     = diag([0.55 0.55].^2);
cfg.track.P0          = diag([1 1 4 0.5 0.5].^2);

%% ---- prediction --------------------------------------------------------
cfg.pred.horizon  = 4.0;    % [s]
cfg.pred.dt       = 0.25;   % [s]
cfg.pred.nModes   = 4;      % CV / CTRV / decel / intent-cross
cfg.pred.sig0     = 0.35;   % [m] base positional sigma
cfg.pred.growth   = 0.55;   % [m/s] sigma growth per second (x erraticness)

%% ---- risk grid ---------------------------------------------------------
cfg.grid.res      = 0.25;   % [m/cell] local grid
cfg.grid.ahead    = 36;     % [m] forward extent
cfg.grid.behind   = 20;     % [m]
cfg.grid.side     = 18;     % [m]
cfg.grid.edgeSoft = 2.0;    % [m] "unclear road edge" soft band (India)
cfg.grid.wEdge    = 35;     % cost of driving on the soft shoulder
cfg.grid.wDyn     = 120;    % dynamic-risk weight
cfg.grid.blockThr = 250;    % cost above which a cell is treated as blocked

%% ---- planners ----------------------------------------------------------
cfg.hybrid.res      = 0.60;   % [m] search grid
cfg.hybrid.nTheta   = 24;
cfg.hybrid.stepLen  = 1.6;    % [m] primitive arc length
cfg.hybrid.steers   = [-1 -0.5 -0.2 0 0.2 0.5 1];  % fraction of maxSteer
cfg.hybrid.wSteer   = 0.7;
cfg.hybrid.wSwitch  = 1.5;
cfg.hybrid.wRisk    = 0.020;
cfg.hybrid.goalTolXY= 2.0;
cfg.hybrid.goalTolH = deg2rad(35);
cfg.hybrid.maxNodes = 3000;
cfg.hybrid.goalAhead= 30;     % [m] along route

cfg.local.dMax      = 3.2;    % [m] lateral sampling half-width (no lanes!)
cfg.local.nLat      = 13;
cfg.local.nSpeed    = 5;
cfg.local.blendLen  = 14;     % [m] lateral transition length
cfg.local.horizonS  = 28;     % [m] path length considered
cfg.local.wOff      = 1.2;    % cost: lateral offset from reference
cfg.local.wRisk     = 0.9;
cfg.local.wSmooth   = 40;     % cost: integral of curvature^2
cfg.local.wSpeed    = 2.5;    % cost: speed shortfall
cfg.local.wJerk     = 0.6;
cfg.local.safetyR   = 0.45;   % [m] extra buffer on ego discs
cfg.local.failsToAStar = 2;   % consecutive failures -> hybrid A* replan

%% ---- behaviour FSM thresholds -----------------------------------------
cfg.fsm.ttcBrake    = 2.4;    % [s]
cfg.fsm.ttcWarn     = 4.0;
cfg.fsm.followGapT  = 1.6;    % [s] time headway
cfg.fsm.creepSpeed  = 1.8;    % [m/s] market / dense crowd
cfg.fsm.yieldSpeed  = 2.5;    % [m/s]
cfg.fsm.mergeGapT   = 2.2;    % [s] required gap to merge
cfg.fsm.holdClearR  = 6.0;    % [m] cattle/VRU clearance to resume
cfg.fsm.densityCreep= 5;      % #agents within 15 m -> CREEP
cfg.fsm.stopDist    = 0.5;    % [m/s] speed under which we call it stopped

%% ---- safety / metrics --------------------------------------------------
cfg.safe.hardBuffer = 0.30;   % [m] collision if gap < this
cfg.safe.minClear   = 0.60;   % [m] desired minimum clearance
cfg.metrics.latencyBudget = 0.100;  % [s] replanning latency requirement

%% ---- IO ----------------------------------------------------------------
cfg.io.outDir   = fullfile(here,'results');
cfg.io.xodrDir  = fullfile(here,'scenes');
cfg.io.video    = true;
cfg.io.videoFPS = 20;
cfg.io.plot     = true;      % live figure
cfg.io.verbose  = true;

%% ---- RoadRunner integration -------------------------------------------
cfg.rr.enable        = true;               % try RoadRunner/ADT if installed
cfg.rr.installFolder = '';                 % e.g. 'C:\Program Files\RoadRunner R2024a'
cfg.rr.projectFolder = fullfile(here,'RRProject');
cfg.rr.sceneFiles    = {'IndianVillageRoad.rrscene','IndianUrbanIntersection.rrscene'};
cfg.rr.scenarioFiles = {'VillageRoad.rrscenario','UrbanIntersection.rrscenario', ...
                        'HighwayMerge.rrscenario','MarketDense.rrscenario', ...
                        'CattleCrossing.rrscenario'};
cfg.rr.xodrFiles     = {'IndianVillageRoad.xodr','IndianUrbanIntersection.xodr'};
cfg.rr.egoActorNames = {};                 % names of the ego actor in your
                                            % .rrscenario files, if you also
                                            % placed one there - these are
                                            % excluded from the traffic read
                                            % back by iadpLoadRoadRunner,
                                            % since the ego is always driven
                                            % by this pipeline's own model.

% used only by iadpOpenInRoadRunner (live, fully-automatic playback):
cfg.rr.egoAssetName    = 'Sedan';          % a Vehicle asset in your project
cfg.rr.egoBehaviorName = 'IADP Ego';       % the one-time custom Behavior asset
                                            % (see IADPEgoBehavior.m header)
cfg.rr.actorAssetMap = struct( ...          % class -> project asset name
    'car','Sedan', 'bus','CityBus', 'truck','Truck', ...
    'autorickshaw','Sedan', 'twowheeler','Sedan', 'bicycle','Sedan', ...
    'pedestrian','Pedestrian', 'animal','Pedestrian', 'pushcart','TrafficCone01', ...
    'default','Sedan');                    % fallback if a class has no mapping
                                            % - override entries with the
                                            % actual asset names in your
                                            % RoadRunner project library.

%% ---- toolbox detection (graceful degradation) --------------------------
cfg.has.adt        = ~isempty(ver('driving'));
cfg.has.nav        = ~isempty(ver('nav'));
cfg.has.dl         = ~isempty(ver('nnet'));
cfg.has.stateflow  = ~isempty(ver('stateflow'));
cfg.has.simulink   = ~isempty(ver('simulink'));
cfg.has.rrapi      = exist('roadrunner','file')==2 || exist('roadrunner','file')==6;

%% ---- name/value overrides ---------------------------------------------
for k = 1:2:numel(varargin)
    cfg = setNested(cfg, varargin{k}, varargin{k+1});
end

if ~exist(cfg.io.outDir,'dir');  mkdir(cfg.io.outDir);  end
if ~exist(cfg.io.xodrDir,'dir'); mkdir(cfg.io.xodrDir); end

rng(cfg.rngSeed);

if cfg.io.verbose
    fprintf('[iadp] Indian Adaptive Driving Planner configured.\n');
    fprintf('[iadp]   Automated Driving Toolbox : %s\n', tf(cfg.has.adt));
    fprintf('[iadp]   Navigation Toolbox        : %s\n', tf(cfg.has.nav));
    fprintf('[iadp]   Deep Learning Toolbox     : %s\n', tf(cfg.has.dl));
    fprintf('[iadp]   Stateflow / Simulink      : %s / %s\n', tf(cfg.has.stateflow), tf(cfg.has.simulink));
    fprintf('[iadp]   RoadRunner API            : %s\n', tf(cfg.has.rrapi));
    fprintf('[iadp]   (missing items use built-in fallbacks)\n');
end
end

% ------------------------------------------------------------------------
function s = setNested(s, name, val)
parts = strsplit(name,'.');
if numel(parts)==1
    s.(parts{1}) = val;
else
    sub = s.(parts{1});
    s.(parts{1}) = setNested(sub, strjoin(parts(2:end),'.'), val);
end
end

function s = tf(b)
if b, s = 'yes'; else, s = 'no (fallback)'; end
end
