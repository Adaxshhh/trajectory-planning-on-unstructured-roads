% IndianADP - Adaptive Path Planning for Unstructured Indian Road Conditions
% Version 1.0  (MATLAB-only implementation, toolbox-optional)
%
% ------------------------------------------------------------------------
% QUICK START
% ------------------------------------------------------------------------
%   >> addpath(pwd)
%   >> iadpTestSuite            % 36 self-tests: units + 5 scenario runs
%   >> R = iadpRunAll;          % full validation campaign + AVI + PNG + report
%
%   Faster campaign (skip video encoding):
%   >> R = iadpRunAll('video',false);
%
%   Single scenario:
%   >> R = iadpRunAll('scenarios',{'CattleCrossing'});
%
%   Dynamic (single-track) plant instead of kinematic bicycle:
%   >> R = iadpRunAll('model','dynamic');
%
%   Everything lands in ./results :
%     <scenario>.avi           demonstration video (perception + plan + actors)
%     <scenario>_metrics.png   speed, latency, clearance, TTC, FSM state plots
%     iadp_results.mat         all logs and metrics structs
%     iadp_report.txt          technical summary + requirement coverage matrix
%
% ------------------------------------------------------------------------
% THE FIVE REQUIRED SCENARIOS  (see iadpScenarioLibrary)
% ------------------------------------------------------------------------
%   VillageRoad          unmarked rural road, no lane markings, oncoming
%                        truck forces a shoulder excursion
%   UrbanIntersection    4-way junction with NO signals, cross traffic,
%                        auto-rickshaws and pedestrians, ego must yield
%   HighwayMerge         ramp merge into slow trucks and two-wheelers,
%                        informal merging with no signalling
%   MarketDense          dense mixed traffic: pushcarts, pedestrians,
%                        bicycles, wandering agents; ego must CREEP
%   CattleCrossing       cattle enters the corridor suddenly; ego must
%                        stop or swerve without collision
%
% ------------------------------------------------------------------------
% ROADRUNNER / OPENDRIVE INPUT
% ------------------------------------------------------------------------
% The pipeline resolves its road network in this order (iadpLoadRoadRunner):
%   1. RoadRunner API  - if RoadRunner and its MATLAB API are installed,
%                        the scene in cfg.rr.sceneFiles is opened and
%                        exported to OpenDRIVE automatically.
%   2. Existing .xodr  - any file you drop in ./scenes is parsed directly.
%   3. Generated .xodr - iadpWriteSampleXODR writes three standards-
%                        compliant OpenDRIVE 1.6 files so the code runs
%                        with zero external assets.
%
% To use YOUR OWN RoadRunner assets:
%   * put  <name>.xodr        in  ./scenes
%   * put  <name>.rrscene     in  ./RRProject/Scenes
%   * put  <name>.rrscenario  in  ./RRProject/Scenarios
%   * point cfg.rr.installFolder at your RoadRunner installation, e.g.
%       cfg = iadpSetup('rr.installFolder','C:\Program Files\RoadRunner R2024a');
%   * actor trajectories may also be supplied as  ./scenes/<name>_actors.csv
%     with columns: id,class,x,y,heading,speed,behavior
%
% OpenDRIVE geometry supported: line, arc, spiral (numeric clothoid),
% poly3 and paramPoly3, with cubic lane-width polynomials.
%
% ------------------------------------------------------------------------
% TOOLBOXES
% ------------------------------------------------------------------------
% Nothing here is required. Every toolbox is detected at run time
% (cfg.has.*) and a built-in equivalent is used when it is absent:
%   Automated Driving Toolbox  -> own sensor models + EKF-CTRV tracker
%   Navigation Toolbox         -> own hybrid A* with a binary heap
%   Stateflow                  -> explicit MATLAB FSM (iadpBehaviorFSM);
%                                 iadpToStateflow builds the real chart
%                                 when Stateflow is installed
%   Simulink / Vehicle Dynamics-> RK4 bicycle model in iadpVehicleStep;
%                                 iadpBuildSimulink generates IADP_Plant.slx
%   Deep Learning Toolbox      -> camera classifier is modelled
%                                 probabilistically (confusion + Pd curves)
%   Image Processing / Stats   -> not used at all (own conv2 dilation,
%                                 own Poisson sampler)
%
% ------------------------------------------------------------------------
% FILE INDEX
% ------------------------------------------------------------------------
% Entry points
%   iadpRunAll           - run the whole campaign, write report and media
%   iadpTestSuite        - 30 unit tests + 5 scenario tests + campaign test
%   iadpSetup            - every tunable parameter in one config struct
%
% Scene and scenario
%   iadpWriteSampleXODR  - generate three valid OpenDRIVE files
%   iadpImportOpenDRIVE  - .xodr parser -> centrelines, lane edges, shoulders
%   iadpBuildMap         - occupancy/cost grid, route extraction, left-lane bias
%   iadpLoadRoadRunner   - RoadRunner API / .xodr / CSV scene+scenario loader
%   iadpScenarioLibrary  - the five Indian scenarios and their pass criteria
%   iadpClassParams      - physical and behavioural parameters per road-user class
%   iadpMakeActor        - construct a ground-truth traffic agent
%   iadpStepActors       - agent behaviours: weave, cutin, cross, merge, wander...
%
% Perception and prediction
%   iadpSensors          - camera / LiDAR / radar with FOV, occlusion,
%                          miss rate, class confusion, clutter
%   iadpFusion           - EKF-CTRV multi-object tracker, GNN association,
%                          chi-squared gating, fused classification
%   iadpPredict          - 4-mode short-term prediction (CTRV, CV, braking,
%                          intent) with class-dependent uncertainty growth
%
% Planning, decision, control, plant
%   iadpRiskGrid         - time-resolved risk field from predictions
%   iadpLocalPlanner     - quintic lateral-offset lattice, speed profile,
%                          collision resolution, cost ranking
%   iadpHybridAStar      - fallback kinodynamic search over the risk grid
%   iadpBehaviorFSM      - 8-state decision logic (CRUISE FOLLOW OBSTACLE
%                          YIELD MERGE CREEP HOLD ESTOP)
%   iadpController       - speed-scheduled pure pursuit + PI speed control
%   iadpVehicleStep      - RK4 kinematic bicycle and dynamic single-track
%   iadpSimulate         - the closed loop, at cfg.dt, with full logging
%
% Results
%   iadpMetrics          - replanning latency (mean/p95/max), curvature and
%                          jerk RMS, min clearance, min TTC, completion rate,
%                          classification accuracy, per-scenario pass/fail
%   iadpVisualize        - AVI demonstration video + 6-panel metrics figure
%
% Export helpers
%   iadpToStateflow      - build the equivalent Stateflow chart
%   iadpBuildSimulink    - build IADP_Plant.slx (controller + plant)
%
% Small utilities
%   iadpMakePath         - resample a polyline to s / heading / curvature
%   iadpPathProject      - project a pose onto a path (Frenet s, lateral d)
%   iadpW2G, iadpG2W     - world <-> grid coordinate conversion
%   iadpEgoDiscs         - 3-disc ego footprint for fast collision checks
%   iadpRectDist         - exact oriented-rectangle distance (SAT) for metrics
%
% ------------------------------------------------------------------------
% DESIGN NOTES
% ------------------------------------------------------------------------
% * Planning is done on continuous lateral offsets, not discrete lanes,
%   because lane discipline cannot be assumed on Indian roads.
% * The road shoulder is a soft cost, not a hard wall; the FSM may grant
%   allowShoulder so the ego can do what local drivers do.
% * Tracking uses a constant turn-rate and velocity (CTRV) EKF rather than
%   constant velocity: mixed traffic turns almost continuously.
% * Prediction blends a physics mode with an intent mode, since a pedestrian
%   or cow stepping off the kerb is not predictable from kinematics alone.
% * Ego keeps left; routes are built by offsetting the centreline to the
%   near-side driving lane.
% * Replanning runs at cfg.planRate (10 Hz) inside a 20 Hz control loop;
%   measured latency is reported against the cfg.budgetMs budget.
%
% See also IADPRUNALL, IADPTESTSUITE, IADPSETUP.
