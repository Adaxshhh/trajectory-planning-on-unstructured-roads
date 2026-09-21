function out = iadpToStateflow(cfg, modelName)
%IADPTOSTATEFLOW  Generate the Stateflow chart of the behaviour layer.
%
%   out = iadpToStateflow(cfg)
%   out = iadpToStateflow(cfg, 'IADP_Behavior')
%
%   If Stateflow is installed this builds a real chart with the eight
%   states and the same guards used by iadpBehaviorFSM, saves it as
%   <modelName>.slx and returns the model name. If Stateflow is not
%   available it prints the chart specification (states, guards, outputs)
%   so it can be reproduced by hand, and returns the spec struct.
%
%   The MATLAB FSM remains the executable one used by the simulation;
%   this exists so the decision logic can be reviewed, documented and
%   code-generated in the standard MathWorks toolchain.

if nargin < 1, cfg = iadpSetup; end
if nargin < 2, modelName = 'IADP_Behavior'; end

spec = chartSpec(cfg);

if ~cfg.has.stateflow || ~cfg.has.simulink
    printSpec(spec);
    out = spec;
    return;
end

try
    sfnew(modelName);
    rt = sfroot;
    ch = rt.find('-isa','Stateflow.Chart','-and','Path',[modelName '/Chart']);
    ch.Name = 'BehaviorPlanner';

    % --- inputs / outputs ---------------------------------------------
    for nm = {'ttc','gapAhead','leadDist','leadSpeed','density','vRoad', ...
              'blockDist','vruDist','conflictDist','mergeGapT','planFail'}
        d = Stateflow.Data(ch); d.Name = nm{1}; d.Scope = 'Input';
    end
    for nm = {'vTarget','latBias','allowShoulder','stateId'}
        d = Stateflow.Data(ch); d.Name = nm{1}; d.Scope = 'Output';
    end

    % --- states ---------------------------------------------------------
    x0 = 40; y0 = 40; w = 190; h = 80; gap = 30;
    S = struct([]);
    for i = 1:numel(spec.states)
        s = Stateflow.State(ch);
        s.Name = spec.states{i};
        s.LabelString = sprintf('%s\nen,du:\n vTarget = %s;\n latBias = %s;\n allowShoulder = %d;\n stateId = %d;', ...
            spec.states{i}, spec.vTarget{i}, spec.latBias{i}, spec.shoulder(i), i);
        col = mod(i-1,3); row = floor((i-1)/3);
        s.Position = [x0 + col*(w+gap), y0 + row*(h+gap), w, h];
        S(i).h = s;
    end

    % --- default transition into CRUISE ---------------------------------
    dt = Stateflow.Transition(ch);
    dt.Destination = S(1).h;
    dt.DestinationOClock = 0;
    dt.SourceEndPoint = S(1).h.Position(1:2) + [20 -30];

    % --- guarded transitions --------------------------------------------
    for i = 1:numel(spec.trans)
        tr = Stateflow.Transition(ch);
        tr.Source      = S(spec.trans(i).from).h;
        tr.Destination = S(spec.trans(i).to).h;
        tr.LabelString = spec.trans(i).guard;
    end

    save_system(modelName, fullfile(cfg.io.outDir, [modelName '.slx']));
    fprintf('[iadp] Stateflow chart written to %s\n', ...
        fullfile(cfg.io.outDir, [modelName '.slx']));
    out = modelName;
catch ME
    warning('iadp:sf','Stateflow generation failed (%s). Printing the spec instead.', ME.message);
    printSpec(spec);
    out = spec;
end
end

% =======================================================================
function spec = chartSpec(cfg)
spec.states  = {'CRUISE','FOLLOW','OBSTACLE','YIELD','MERGE','CREEP','HOLD','ESTOP'};
spec.vTarget = {'vRoad', ...
                'min(vRoad, leadSpeed + 0.55*(leadDist - followGapT*v))', ...
                'min(vRoad, 5.0)', ...
                sprintf('%.1f', cfg.fsm.yieldSpeed), ...
                'mergeGapT > mergeGapReq ? vRoad : mergeAgentSpeed-2', ...
                sprintf('%.1f', cfg.fsm.creepSpeed), ...
                '0', '0'};
spec.latBias = {'0','0','1.2*freeSide','0','-0.6*mergeSide','0.4*freeSide','0','0'};
spec.shoulder= [0 0 1 0 0 1 1 1];

g = {};
g{end+1} = mk(1:8, 8, sprintf('[ttc < %.2f || gapAhead < 2.5]', cfg.fsm.ttcBrake));
g{end+1} = mk(1:8, 3, '[planFail || (blockDist < 30 && blockStatic)]');
g{end+1} = mk(1:8, 7, '[vruInCorridor && vruDist < 18]');
g{end+1} = mk(1:8, 4, '[conflictAgent && conflictDist < 28]');
g{end+1} = mk(1:8, 6, sprintf('[density >= %d]', cfg.fsm.densityCreep));
g{end+1} = mk(1:8, 5, '[mergeAgent]');
g{end+1} = mk(1:8, 2, sprintf('[leadDist < max(12, %.1f*v*1.6)]', cfg.fsm.followGapT));
g{end+1} = mk([2 3 4 5 6 7 8], 1, '[else]');

T = struct('from',{},'to',{},'guard',{});
for i = 1:numel(g)
    for f = g{i}.from
        if f == g{i}.to, continue; end
        T(end+1) = struct('from',f,'to',g{i}.to,'guard',g{i}.guard); %#ok<AGROW>
    end
end
spec.trans = T;
spec.priorityNote = ['Transitions are evaluated in the order listed; ' ...
                     'ESTOP has the highest priority and CRUISE the lowest.'];
end

function s = mk(from, to, guard)
s.from = from; s.to = to; s.guard = guard;
end

function printSpec(spec)
fprintf('\n--- Stateflow chart specification (behaviour layer) ---\n');
for i = 1:numel(spec.states)
    fprintf('  state %-9s : vTarget = %-58s latBias = %-18s shoulder = %d\n', ...
        spec.states{i}, spec.vTarget{i}, spec.latBias{i}, spec.shoulder(i));
end
fprintf('  %s\n', spec.priorityNote);
fprintf('  %d guarded transitions generated.\n\n', numel(spec.trans));
end
