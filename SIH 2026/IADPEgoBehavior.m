classdef IADPEgoBehavior < matlab.System
%IADPEGOBEHAVIOR  Live RoadRunner Scenario actor behavior for the ego
%vehicle. Runs the SAME perception -> fusion -> prediction -> risk grid
%-> FSM -> local planner -> (hybrid A* fallback) -> controller -> bicycle
%model pipeline as iadpSimulate.m, but instead of stepping a self-made
%actor list, it reads every OTHER actor's live pose straight out of the
%running RoadRunner Scenario each tick, and instead of logging to a
%MATLAB array it writes the ego's new pose back into RoadRunner every
%tick - so you watch it steer, brake and dodge inside RoadRunner itself,
%with the traffic being whatever you placed and animated in RoadRunner
%Scenario Editor.
%
%   SET UP IN ROADRUNNER SCENARIO EDITOR (one-time, per scene)
%     1. Open your scene's .rrscenario in RoadRunner Scenario.
%     2. Library Browser > Behaviors > right-click > New > Behavior.
%        Name it e.g. "IADP Ego". Attributes pane: Platform =
%        MATLAB/Simulink, File Name = the full path to this file
%        (IADPEgoBehavior.m).
%     3. Select your ego actor (a car), Inspector > Behavior > choose
%        "IADP Ego". In the same Inspector you can now edit this
%        behavior's public properties listed below (ScenarioName, etc).
%     4. Leave every OTHER actor's own behavior/path as you authored it
%        (Smart Actor, Path Following, whatever) - that is your traffic,
%        read live every tick by this file.
%     5. From MATLAB:
%          rrApp = roadrunner(cfg.rr.projectFolder, ...
%                              'InstallationFolder', cfg.rr.installFolder);
%          openScenario(rrApp, 'VillageRoad');           % your .rrscenario
%          sim = createSimulation(rrApp);
%          set(sim, 'SimulationCommand', 'Start');
%        RoadRunner now plays live and you can watch the ego actor move.
%
%   PUBLIC PROPERTIES (edit these in the RoadRunner Inspector, or set
%   them on the object before Start if you are driving it from a
%   Simulink model instead of directly from the Behavior attribute):
%     ScenarioName   - which row of iadpScenarioLibrary.m to use for the
%                       road network and ego start/goal (default 'VillageRoad')
%     OtherActorNames- cell array of the names of the OTHER actors in this
%                       .rrscenario to treat as traffic. Leave empty to
%                       attempt automatic discovery (reads every actor in
%                       the scenario except this one); set explicitly if
%                       that automatic read errors on your RoadRunner
%                       version (this is the one part of this file that
%                       could not be executed and verified in advance -
%                       see the try/catch in otherActorPoses below).
%     LogFile        - where to save a .mat trace of the run for offline
%                       plotting with iadpMetrics-style stats (default
%                       results/rrcosim_<ScenarioName>.mat, '' to disable)
%
%   This file mirrors iadpSimulate.m's per-tick pipeline exactly (same
%   function calls, same order) so behaviour matches the offline runs
%   from iadpRunAll. It does not call iadpStepActors - the traffic's
%   motion is entirely RoadRunner's, read fresh every tick.

    properties
        ScenarioName    char   = 'VillageRoad'
        OtherActorNames cell   = {}
        LogFile         char   = ''
    end

    properties (Access = private)
        mScenarioSimulationHdl
        mActorSimulationHdl
        mStepSize
        mCfg
        mMap
        mGoal
        mRefPath
        mRefIsAStar   = false
        mFailCount    = 0
        mTrk
        mFsm
        mCtl
        mTraj
        mCmd
        mEgo
        mLastTime  = 0
        mTickCount = 0
        mPlanEvery = 1
        mLog       % struct of growable arrays for the LogFile trace
        mLogN      = 0
    end

    methods (Access = protected)

        function st = getSampleTimeImpl(obj)
            obj.mScenarioSimulationHdl = Simulink.ScenarioSimulation.find('ScenarioSimulation');
            obj.mStepSize = obj.mScenarioSimulationHdl.get('StepSize');
            st = createSampleTime(obj, 'Type','Discrete','SampleTime',obj.mStepSize);
        end

        function sz = getOutputSizeImpl(~)
            sz = [1 1];
        end
        function t = getOutputDataTypeImpl(~)
            t = 'double';
        end
        function cp = isOutputComplexImpl(~)
            cp = false;
        end
        function fz = isOutputFixedSizeImpl(~)
            fz = true;
        end

        function setupImpl(obj)
            obj.mActorSimulationHdl = Simulink.ScenarioSimulation.find( ...
                'ActorSimulation', 'SystemObject', obj);

            obj.mCfg  = iadpSetup('io.video', false, 'io.verbose', false);
            spec = iadpScenarioLibrary(obj.mCfg, obj.ScenarioName);

            % ---- build the map from the SAME xodrFile/rrsceneFile the
            % offline pipeline would use, via a tiny local re-use of the
            % scene half of iadpLoadRoadRunner's resolution order.
            xodr = '';
            if ~isempty(spec.xodrFile)
                xodr = fullfile(obj.mCfg.io.xodrDir, [spec.xodrFile '.xodr']);
            end
            if isempty(xodr) || ~exist(xodr,'file')
                error('iadp:rrcosim:noScene', [ ...
                    'IADPEgoBehavior needs scenes/%s.xodr for scenario "%s".\n' ...
                    'Export it from RoadRunner (File > Export > ASAM OpenDRIVE)\n' ...
                    'and place it there, or fix xodrFile in iadpScenarioLibrary.m.'], ...
                    spec.xodrFile, obj.ScenarioName);
            end
            odr = iadpImportOpenDRIVE(xodr, 0.5);
            obj.mMap = iadpBuildMap(odr, obj.mCfg, spec.routeIdx, spec.startXY);
            obj.mRefPath = obj.mMap.route;
            obj.mGoal = struct('s', min(obj.mMap.route.len - 3, spec.goalS), 'tol', 4.0);

            obj.mTrk = iadpFusion('init', obj.mCfg);
            obj.mFsm = [];
            obj.mCtl = struct('eInt',0);
            obj.mTraj = [];
            obj.mCmd  = struct('vTarget',spec.egoV0,'latBias',0,'allowShoulder',false,'mode','CRUISE');

            pose = obj.mActorSimulationHdl.getAttribute('Pose');
            vel  = obj.mActorSimulationHdl.getAttribute('Velocity');
            psi0 = atan2(pose(2,1), pose(1,1));
            obj.mEgo = struct('x',pose(1,4), 'y',pose(2,4), 'psi',psi0, ...
                'v',norm(vel(1:2)), 'delta',0, 'beta',0, 'r',0, 'a',0);

            obj.mPlanEvery = max(1, round(1/(obj.mCfg.planRate*obj.mStepSize)));
            obj.mLastTime  = 0;
            obj.mTickCount = 0;

            if isempty(obj.LogFile)
                obj.LogFile = fullfile(obj.mCfg.io.outDir, ...
                    sprintf('rrcosim_%s.mat', obj.ScenarioName));
            end
            cap = 200000;
            obj.mLog = struct('t',zeros(cap,1),'x',zeros(cap,1),'y',zeros(cap,1), ...
                'psi',zeros(cap,1),'v',zeros(cap,1),'mode',cell(cap,1), ...
                'minClear',inf(cap,1));
            obj.mLogN = 0;
        end

        function y = stepImpl(obj, ~)
            t = obj.mTickCount * obj.mStepSize;
            obj.mTickCount = obj.mTickCount + 1;

            % ---- sync ego from RR in case anything nudged it externally
            pose = obj.mActorSimulationHdl.getAttribute('Pose');
            vel  = obj.mActorSimulationHdl.getAttribute('Velocity');
            obj.mEgo.x = pose(1,4); obj.mEgo.y = pose(2,4);
            obj.mEgo.psi = atan2(pose(2,1), pose(1,1));
            obj.mEgo.v = norm(vel(1:2));

            actors = otherActorPoses(obj);

            if mod(obj.mTickCount-1, obj.mPlanEvery) == 0
                cfg = obj.mCfg;
                dets = iadpSensors(cfg, obj.mEgo, actors, t);
                [obj.mTrk, tracks] = iadpFusion('update', obj.mTrk, dets, obj.mPlanEvery*obj.mStepSize, cfg);

                preds = iadpPredict(tracks, cfg, obj.mMap, obj.mEgo);
                G     = iadpRiskGrid(obj.mMap, cfg, obj.mEgo, preds);

                [obj.mFsm, obj.mCmd] = iadpBehaviorFSM(obj.mFsm, cfg, obj.mEgo, tracks, preds, ...
                    obj.mMap, t, struct('fail', obj.mFailCount>0));

                [trajNew, pinfo] = iadpLocalPlanner(obj.mRefPath, obj.mEgo, preds, G, cfg, obj.mCmd);
                if pinfo.fail
                    obj.mFailCount = obj.mFailCount + 1;
                else
                    obj.mFailCount = 0;
                    obj.mTraj = trajNew;
                end

                needAStar = obj.mFailCount >= cfg.local.failsToAStar || ...
                    (strcmp(obj.mCmd.mode,'OBSTACLE') && ~obj.mRefIsAStar && obj.mFsm.changed);
                if needAStar
                    [sE, ~, ~] = iadpPathProject(obj.mMap.route, obj.mEgo.x, obj.mEgo.y);
                    sG = min(sE + cfg.hybrid.goalAhead, obj.mMap.route.len);
                    iG = find(obj.mMap.route.s >= sG, 1, 'first');
                    if isempty(iG), iG = numel(obj.mMap.route.s); end
                    goalPose = [obj.mMap.route.xy(iG,1) obj.mMap.route.xy(iG,2) obj.mMap.route.hdg(iG)];
                    [apath, ~] = iadpHybridAStar(G, cfg, [obj.mEgo.x obj.mEgo.y obj.mEgo.psi], goalPose);
                    if ~isempty(apath) && size(apath,1) > 3
                        obj.mRefPath = iadpMakePath(apath(:,1:2), 0.5);
                        obj.mRefIsAStar = true;
                        obj.mFailCount = 0;
                        obj.mCmd.latBias = 0;
                        [trajNew, pinfo] = iadpLocalPlanner(obj.mRefPath, obj.mEgo, preds, G, cfg, obj.mCmd);
                        if ~pinfo.fail, obj.mTraj = trajNew; end
                    end
                end
                if obj.mRefIsAStar
                    [sR, ~] = iadpPathProject(obj.mRefPath, obj.mEgo.x, obj.mEgo.y);
                    if obj.mRefPath.len - sR < 6
                        obj.mRefPath = obj.mMap.route; obj.mRefIsAStar = false;
                    end
                end

                if obj.mLogN < numel(obj.mLog.t)
                    [mc, ~, ~] = clearanceLocal(obj.mEgo, actors, cfg);
                    obj.mLogN = obj.mLogN + 1;
                    obj.mLog.t(obj.mLogN) = t; obj.mLog.x(obj.mLogN) = obj.mEgo.x;
                    obj.mLog.y(obj.mLogN) = obj.mEgo.y; obj.mLog.psi(obj.mLogN) = obj.mEgo.psi;
                    obj.mLog.v(obj.mLogN) = obj.mEgo.v; obj.mLog.mode{obj.mLogN} = obj.mCmd.mode;
                    obj.mLog.minClear(obj.mLogN) = mc;
                end
            end

            % ---- control + plant, every tick (zero-order hold on cmd) --
            [delta, aCmd, obj.mCtl] = iadpController(obj.mEgo, obj.mTraj, obj.mCfg, obj.mCtl, obj.mStepSize);
            obj.mEgo = iadpVehicleStep(obj.mEgo, delta, aCmd, obj.mCfg, obj.mStepSize);

            % ---- write the new pose back so RoadRunner renders it -------
            newPose = pose;
            newPose(1,4) = obj.mEgo.x; newPose(2,4) = obj.mEgo.y;
            c = cos(obj.mEgo.psi); sN = sin(obj.mEgo.psi);
            newPose(1:2,1:2) = [c -sN; sN c];
            newVel = [obj.mEgo.v*c, obj.mEgo.v*sN, 0];
            obj.mActorSimulationHdl.setAttribute('Pose', newPose);
            obj.mActorSimulationHdl.setAttribute('Velocity', newVel);

            y = t;
        end

        function releaseImpl(obj)
            if ~isempty(obj.LogFile) && obj.mLogN > 0
                n = obj.mLogN;
                L.t = obj.mLog.t(1:n); L.x = obj.mLog.x(1:n); L.y = obj.mLog.y(1:n);
                L.psi = obj.mLog.psi(1:n); L.v = obj.mLog.v(1:n);
                L.mode = obj.mLog.mode(1:n); L.minClear = obj.mLog.minClear(1:n);
                try
                    save(obj.LogFile, 'L');
                catch ME
                    warning('iadp:rrcosim','could not write LogFile %s (%s)', obj.LogFile, ME.message);
                end
            end
        end
    end

    methods (Access = private)
        function actors = otherActorPoses(obj)
        %OTHERACTORPOSES  Live traffic read from RoadRunner, converted to
        %the same actor struct iadpSensors/iadpMakeActor expect. Primary
        %path enumerates every actor in the scenario and skips this one;
        %if that call is not supported on your RoadRunner/MATLAB release,
        %set OtherActorNames explicitly and it will look each one up by
        %name instead - that call is the one confirmed by MathWorks'
        %ActorSimulation.find(...,'Name',...) documentation.
        actors = [];
        id = 0;
        try
            w = get(obj.mScenarioSimulationHdl, 'Actors');
            for k = 1:numel(w)
                a = w(k);
                if isfield(a,'Name') && isprop(obj.mActorSimulationHdl,'Name') ...
                        && strcmp(char(a.Name), obj.mActorSimulationHdl.Name)
                    continue
                end
                id = id + 1;
                actors = [actors iadpMakeActor(id, mapRRClassLocal(a.ActorType), ...
                    a.Pose(1,4), a.Pose(2,4), atan2(a.Pose(2,1),a.Pose(1,1)), ...
                    norm(a.Velocity(1:2)), 'constant', 0.5)]; %#ok<AGROW>
            end
            if ~isempty(actors), return; end
        catch
            % fall through to the name-list method below
        end

        for k = 1:numel(obj.OtherActorNames)
            try
                h = Simulink.ScenarioSimulation.find('ActorSimulation', 'Name', obj.OtherActorNames{k});
                p = h.getAttribute('Pose'); v = h.getAttribute('Velocity');
                id = id + 1;
                actors = [actors iadpMakeActor(id, 'car', p(1,4), p(2,4), ...
                    atan2(p(2,1),p(1,1)), norm(v(1:2)), 'constant', 0.5)]; %#ok<AGROW>
            catch ME
                warning('iadp:rrcosim','could not read actor "%s" (%s)', ...
                    obj.OtherActorNames{k}, ME.message);
            end
        end
        end
    end
end

% =======================================================================
function c = mapRRClassLocal(t)
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
function [mc, who, ttc] = clearanceLocal(ego, actors, cfg)
mc = inf; who = ''; ttc = inf;
ep = [ego.x + (cfg.veh.length/2 - cfg.veh.rearOH)*cos(ego.psi), ...
      ego.y + (cfg.veh.length/2 - cfg.veh.rearOH)*sin(ego.psi), ego.psi];
for i = 1:numel(actors)
    a = actors(i);
    if isfield(a,'alive') && ~a.alive, continue; end
    d = iadpRectDist(ep, [cfg.veh.length cfg.veh.width], ...
                     [a.x a.y a.psi], [a.p.L a.p.W]);
    if d < mc, mc = d; who = sprintf('%s#%d', a.cls, a.id); end
end
end
