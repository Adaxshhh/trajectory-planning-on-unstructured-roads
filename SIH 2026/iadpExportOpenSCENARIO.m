function xosc = iadpExportOpenSCENARIO(res, xodrPath, outFile, dtOut)
%IADPEXPORTOPENSCENARIO  Write the simulated run (ego + every traffic
%actor, as logged by iadpSimulate) as an ASAM OpenSCENARIO 1.0 file.
%RoadRunner Scenario imports it (importScenario) and plays every actor
%along its recorded trajectory - so the ego's detect-and-dodge manoeuvres
%computed by the MATLAB pipeline are what you watch in RoadRunner.
%
%   xosc = iadpExportOpenSCENARIO(res, xodrPath, outFile)
%   xosc = iadpExportOpenSCENARIO(res, xodrPath, outFile, 0.1)   % sample period [s]
%
%   RoadRunner requires every entity to have an initial SpeedAction in
%   <Init>; this is written from the first two trajectory samples.

if nargin < 4, dtOut = 0.1; end
L  = res.log;
t  = L.t(:);
tq = (0:dtOut:t(end)).';
if numel(tq) < 2, tq = [0; t(end)+dtOut]; end

names = {'Ego'}; cats = {'car'}; kind = {'vehicle'}; dims = {[4.4 1.8 1.5]};
X = {interp1(t, L.x,   tq, 'linear', 'extrap')};
Y = {interp1(t, L.y,   tq, 'linear', 'extrap')};
H = {unwrap(interp1(t, unwrap(L.psi), tq, 'linear', 'extrap'))};
for i = 1:numel(res.actors)
    a = res.actors(i);
    names{end+1} = sprintf('%s_%d', a.cls, a.id);   %#ok<AGROW>
    [c, k] = mapClass(a.cls);
    cats{end+1} = c; kind{end+1} = k;               %#ok<AGROW>
    dims{end+1} = [a.p.L a.p.W 1.5];                %#ok<AGROW>
    ok = ~isnan(L.ax(:,i));
    X{end+1} = interp1(t(ok), L.ax(ok,i), tq, 'linear', 'extrap');           %#ok<AGROW>
    Y{end+1} = interp1(t(ok), L.ay(ok,i), tq, 'linear', 'extrap');           %#ok<AGROW>
    H{end+1} = unwrap(interp1(t(ok), unwrap(L.apsi(ok,i)), tq, 'linear','extrap')); %#ok<AGROW>
end

% initial speed of every entity from the first trajectory samples
v0 = zeros(1, numel(names));
for i = 1:numel(names)
    kk = min(3, numel(tq));
    v0(i) = hypot(X{i}(kk)-X{i}(1), Y{i}(kk)-Y{i}(1)) / (tq(kk)-tq(1));
    if ~isfinite(v0(i)), v0(i) = 0; end
end
v0(1) = L.v(1);   % ego: use the logged speed

f = fopen(outFile,'w');
if f < 0, error('iadp:xosc','cannot write %s', outFile); end
c = onCleanup(@() fclose(f));
p = @(varargin) fprintf(f, varargin{:});

p('<?xml version="1.0" encoding="UTF-8"?>\n<OpenSCENARIO>\n');
p('  <FileHeader revMajor="1" revMinor="0" date="2026-01-01T00:00:00" description="IADP %s" author="IndianADP"/>\n', res.name);
p('  <ParameterDeclarations/>\n  <CatalogLocations/>\n');
p('  <RoadNetwork><LogicFile filepath="%s"/></RoadNetwork>\n', strrep(xodrPath,'\','/'));
p('  <Entities>\n');
for i = 1:numel(names)
    d = dims{i};
    if strcmp(kind{i},'vehicle')
        p('    <ScenarioObject name="%s"><Vehicle name="%s" vehicleCategory="%s">\n', names{i}, names{i}, cats{i});
        p('      <ParameterDeclarations/>\n');
        p('      <BoundingBox><Center x="%.2f" y="0" z="%.2f"/><Dimensions width="%.2f" length="%.2f" height="%.2f"/></BoundingBox>\n', d(1)/2*0.8, d(3)/2, d(2), d(1), d(3));
        p('      <Performance maxSpeed="40" maxAcceleration="5" maxDeceleration="9"/>\n');
        p('      <Axles><FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="%.2f" positionX="%.2f" positionZ="0.3"/>', d(2)*0.85, d(1)*0.7);
        p('<RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="%.2f" positionX="0" positionZ="0.3"/></Axles>\n', d(2)*0.85);
        p('      <Properties/>\n    </Vehicle></ScenarioObject>\n');
    else
        p('    <ScenarioObject name="%s"><Pedestrian model="%s" mass="70" name="%s" pedestrianCategory="%s">\n', names{i}, cats{i}, names{i}, cats{i});
        p('      <ParameterDeclarations/>\n');
        p('      <BoundingBox><Center x="0" y="0" z="0.9"/><Dimensions width="%.2f" length="%.2f" height="1.8"/></BoundingBox>\n', max(d(2),0.5), max(d(1),0.5));
        p('      <Properties/>\n    </Pedestrian></ScenarioObject>\n');
    end
end
p('  </Entities>\n  <Storyboard>\n    <Init><Actions>\n');
for i = 1:numel(names)
    p('      <Private entityRef="%s">\n', names{i});
    p('        <PrivateAction><TeleportAction><Position>');
    p('<WorldPosition x="%.3f" y="%.3f" z="0" h="%.4f" p="0" r="0"/></Position></TeleportAction></PrivateAction>\n', X{i}(1), Y{i}(1), H{i}(1));
    p('        <PrivateAction><LongitudinalAction><SpeedAction>');
    p('<SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>');
    p('<SpeedActionTarget><AbsoluteTargetSpeed value="%.3f"/></SpeedActionTarget>', v0(i));
    p('</SpeedAction></LongitudinalAction></PrivateAction>\n');
    p('      </Private>\n');
end
p('    </Actions></Init>\n    <Story name="IADPStory"><Act name="IADPAct">\n');
for i = 1:numel(names)
    p('      <ManeuverGroup maximumExecutionCount="1" name="mg_%d"><Actors selectTriggeringEntities="false"><EntityRef entityRef="%s"/></Actors>\n', i, names{i});
    p('        <Maneuver name="m_%d"><Event name="e_%d" priority="overwrite"><Action name="a_%d"><PrivateAction><RoutingAction><FollowTrajectoryAction>\n', i, i, i);
    p('          <Trajectory name="traj_%d" closed="false"><Shape><Polyline>\n', i);
    for k = 1:numel(tq)
        p('            <Vertex time="%.3f"><Position><WorldPosition x="%.3f" y="%.3f" z="0" h="%.4f" p="0" r="0"/></Position></Vertex>\n', tq(k), X{i}(k), Y{i}(k), H{i}(k));
    end
    p('          </Polyline></Shape></Trajectory>\n');
    p('          <TimeReference><Timing domainAbsoluteRelative="absolute" scale="1" offset="0"/></TimeReference>\n');
    p('          <TrajectoryFollowingMode followingMode="position"/>\n');
    p('        </FollowTrajectoryAction></RoutingAction></PrivateAction></Action>\n');
    p('        <StartTrigger><ConditionGroup><Condition name="start_%d" delay="0" conditionEdge="rising"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition></Condition></ConditionGroup></StartTrigger>\n', i);
    p('        </Event></Maneuver></ManeuverGroup>\n');
end
p('      <StartTrigger><ConditionGroup><Condition name="actStart" delay="0" conditionEdge="rising"><ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition></Condition></ConditionGroup></StartTrigger>\n');
p('    </Act></Story>\n');
p('    <StopTrigger><ConditionGroup><Condition name="stop" delay="0" conditionEdge="rising"><ByValueCondition><SimulationTimeCondition value="%.1f" rule="greaterThan"/></ByValueCondition></Condition></ConditionGroup></StopTrigger>\n', tq(end)+1);
p('  </Storyboard>\n</OpenSCENARIO>\n');
xosc = outFile;
end

function [cat, kind] = mapClass(cls)
switch lower(cls)
    case 'bus',          cat = 'bus';       kind = 'vehicle';
    case 'truck',        cat = 'truck';     kind = 'vehicle';
    case 'twowheeler',   cat = 'motorbike'; kind = 'vehicle';
    case 'bicycle',      cat = 'bicycle';   kind = 'vehicle';
    case 'pedestrian',   cat = 'pedestrian';kind = 'ped';
    case 'animal',       cat = 'animal';    kind = 'ped';
    case 'pushcart',     cat = 'wheelchair';kind = 'ped';
    otherwise,           cat = 'car';       kind = 'vehicle';
end
end
