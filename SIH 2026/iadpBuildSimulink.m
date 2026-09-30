function out = iadpBuildSimulink(cfg, modelName)
%IADPBUILDSIMULINK  Build the Simulink vehicle-model loop programmatically.
%
%   out = iadpBuildSimulink(cfg)
%   out = iadpBuildSimulink(cfg, 'IADP_Plant')
%
%   Creates a Simulink model containing:
%     - a MATLAB Function block wrapping iadpVehicleStep (bicycle model,
%       kinematic or dynamic per cfg.veh.model)
%     - a MATLAB Function block wrapping iadpController (pure pursuit + PI)
%     - the closed loop between them, with the planner trajectory supplied
%       from the workspace
%
%   This gives the Simulink-side deliverable while keeping ONE
%   implementation of the dynamics, so the MATLAB campaign and the
%   Simulink model can never diverge. If Simulink is unavailable the
%   function returns the block specification instead of erroring.

if nargin < 1, cfg = iadpSetup; end
if nargin < 2, modelName = 'IADP_Plant'; end

if ~cfg.has.simulink
    out = blockSpec(cfg);
    fprintf(['[iadp] Simulink not available. Model specification:\n' ...
             '       plant  : %s\n       control: %s\n       step   : %.3f s\n'], ...
             out.plant, out.control, cfg.dt);
    return;
end

try
    if bdIsLoaded(modelName), close_system(modelName, 0); end
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Sources/In1',  [modelName '/trajBus']);
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [modelName '/Controller']);
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [modelName '/VehicleModel']);
    add_block('simulink/Discrete/Memory', [modelName '/EgoState']);
    add_block('simulink/Sinks/Out1', [modelName '/egoOut']);

    set_param([modelName '/Controller'], 'Position', [180 100 300 180]);
    set_param([modelName '/VehicleModel'], 'Position', [380 100 520 180]);
    set_param([modelName '/EgoState'], 'Position', [380 240 420 280]);

    setFcn([modelName '/Controller'], controllerCode());
    setFcn([modelName '/VehicleModel'], plantCode());

    add_line(modelName, 'trajBus/1',     'Controller/1', 'autorouting','on');
    add_line(modelName, 'EgoState/1',    'Controller/2', 'autorouting','on');
    add_line(modelName, 'Controller/1',  'VehicleModel/1','autorouting','on');
    add_line(modelName, 'EgoState/1',    'VehicleModel/2','autorouting','on');
    add_line(modelName, 'VehicleModel/1','EgoState/1',   'autorouting','on');
    add_line(modelName, 'VehicleModel/1','egoOut/1',     'autorouting','on');

    set_param(modelName, 'FixedStep', num2str(cfg.dt), ...
                         'Solver','FixedStepDiscrete', ...
                         'StopTime', num2str(cfg.tMax));
    save_system(modelName, fullfile(cfg.io.outDir, [modelName '.slx']));
    fprintf('[iadp] Simulink model written to %s\n', ...
        fullfile(cfg.io.outDir, [modelName '.slx']));
    out = modelName;
catch ME
    warning('iadp:slx','Simulink build failed: %s', ME.message);
    out = blockSpec(cfg);
end
end

% =======================================================================
function setFcn(blk, code)
rt = sfroot;
b  = rt.find('-isa','Stateflow.EMChart','-and','Path',blk);
b.Script = code;
end

function c = controllerCode()
c = sprintf([ ...
 'function u = fcn(traj, ego)\n' ...
 '%%#codegen\n' ...
 'cfg = iadpSetup();\n' ...
 'ctl = struct(''eInt'',0);\n' ...
 '[delta, a] = iadpController(ego, traj, cfg, ctl, cfg.dt);\n' ...
 'u = [delta; a];\n']);
end

function c = plantCode()
c = sprintf([ ...
 'function egoNew = fcn(u, ego)\n' ...
 '%%#codegen\n' ...
 'cfg = iadpSetup();\n' ...
 'egoNew = iadpVehicleStep(ego, u(1), u(2), cfg, cfg.dt);\n']);
end

function s = blockSpec(cfg)
s.plant   = sprintf('bicycle model (%s), L=%.2f m, delta<=%.2f rad, rate<=%.2f rad/s', ...
                    cfg.veh.model, cfg.veh.L, cfg.veh.maxSteer, cfg.veh.maxSteerRt);
s.control = sprintf('pure pursuit (Ld = %.1f..%.1f m) + PI speed (Kp=%.2f, Ki=%.2f)', ...
                    cfg.ctrl.LdMin, cfg.ctrl.LdMax, cfg.ctrl.Kp, cfg.ctrl.Ki);
s.step    = cfg.dt;
end
