function rrApp = iadpPlayInRoadRunner(res, spec, cfg, rrApp)
%IADPPLAYINROADRUNNER  Open RoadRunner (if not already open), build the
%scene + scenario for one finished run and play it. Errors are shown,
%never swallowed.
%   rrApp = iadpPlayInRoadRunner(res, spec, cfg)          % opens RoadRunner
%   rrApp = iadpPlayInRoadRunner(res, spec, cfg, rrApp)   % reuse the session
name = res.name;
xodr = fullfile(cfg.io.xodrDir, [spec.scene '.xodr']);
xosc = fullfile(cfg.io.outDir, [name '.xosc']);
iadpExportOpenSCENARIO(res, xodr, xosc, 0.1);
fprintf('[iadp] wrote %s\n', xosc);

if nargin < 4 || isempty(rrApp)
    rrApp = openRoadRunner(cfg);
end

%% ---- scene ------------------------------------------------------------
newScene(rrApp);
importScene(rrApp, xodr, 'OpenDRIVE');
saveScene(rrApp, [spec.scene '.rrscene']);
fprintf('[iadp] created %s.rrscene\n', spec.scene);

%% ---- scenario ---------------------------------------------------------
newScenario(rrApp);
importScenario(rrApp, xosc, 'OpenSCENARIO');
saveScenario(rrApp, [name '.rrscenario']);
fprintf('[iadp] created %s.rrscenario\n', name);

%% ---- play -------------------------------------------------------------
sim = createSimulation(rrApp);
set(sim, 'SimulationCommand', 'Start');
fprintf('[iadp] playing "%s" in RoadRunner (%.0f s) ...\n', name, res.tEnd);
pause(min(res.tEnd, 90) + 2);
end

% =======================================================================
function rrApp = openRoadRunner(cfg)
%OPENROADRUNNER  Try, in order: cfg.rr.projectFolder (only if it is a real
%RoadRunner project), the defaults saved by roadrunnerSetup, then an
%interactive roadrunnerSetup dialog.
rrApp = [];
pf = cfg.rr.projectFolder;
if exist(fullfile(pf,'Assets'),'dir')
    try
        if isempty(cfg.rr.installFolder)
            rrApp = roadrunner(pf);
        else
            rrApp = roadrunner(pf, 'InstallationFolder', cfg.rr.installFolder);
        end
        fprintf('[iadp] opened RoadRunner project %s\n', pf);
        return;
    catch ME
        fprintf('[iadp] project %s did not open (%s)\n', pf, ME.message);
    end
end
try
    rrApp = roadrunner;            % defaults saved earlier by roadrunnerSetup
    fprintf('[iadp] opened RoadRunner with your saved default project\n');
    return;
catch ME
    fprintf('[iadp] default roadrunner() failed (%s)\n', ME.message);
end
fprintf(['[iadp] No usable RoadRunner project yet. A dialog will open: choose your\n' ...
         '       RoadRunner install folder and a project folder (create one via\n' ...
         '       RoadRunner > File > New Project if needed), and pick "Across MATLAB\n' ...
         '       sessions" so you are never asked again.\n']);
rrApp = roadrunnerSetup;
end
