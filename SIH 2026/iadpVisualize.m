function out = iadpVisualize(res, cfg, what)
%IADPVISUALIZE  Animation / video / result plots for one scenario.
%
%   iadpVisualize(res, cfg, 'video')    writes <name>.avi (demonstration)
%   iadpVisualize(res, cfg, 'plots')    writes <name>_metrics.png
%   iadpVisualize(res, cfg, 'all')      both
%
%   Colours follow class: vehicles blue, rickshaws orange, two-wheelers
%   magenta, pedestrians red, animals brown, carts grey.

if nargin < 3, what = 'all'; end
out = {};
if any(strcmp(what,{'video','all'}))  && ~isempty(res.frames)
    out{end+1} = makeVideo(res, cfg);
end
if any(strcmp(what,{'plots','all'}))
    out{end+1} = makePlots(res, cfg);
end
end

% =======================================================================
function f = makeVideo(res, cfg)
f = fullfile(cfg.io.outDir, sprintf('%s.avi', res.name));
fig = figure('Visible','off','Position',[50 50 1100 620],'Color','w');
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal');

vw = VideoWriter(f, 'Motion JPEG AVI');
vw.FrameRate = cfg.io.videoFPS/2;
open(vw);

map = res.map;
for i = 1:numel(res.frames)
    F = res.frames{i};
    cla(ax); hold(ax,'on');

    drawRoads(ax, map);

    % risk field (coarse)
    if isfield(F,'risk')
        [ny, nx] = size(F.risk);
        xs = F.riskX + (0.5:nx)*F.riskRes;
        ys = F.riskY + (0.5:ny)*F.riskRes;
        rk = min(F.risk, 300);
        im = imagesc(ax, xs, ys, rk);
        set(im,'AlphaData', 0.35*(rk>10));
        set(ax,'YDir','normal');
        uistack(im,'bottom');
    end

    % reference + planned trajectory
    plot(ax, F.ref(:,1), F.ref(:,2), ':', 'Color',[0.3 0.3 0.3], 'LineWidth',1);
    if ~isempty(F.pred)
        plot(ax, F.pred(:,1), F.pred(:,2), '-', 'Color',[0.85 0.4 0.1 ], 'LineWidth',0.7);
    end
    if ~isempty(F.traj)
        plot(ax, F.traj(:,1), F.traj(:,2), '-', 'Color',[0 0.6 0.2], 'LineWidth',2.2);
    end

    % actors
    for a = 1:size(F.actors,1)
        drawRect(ax, F.actors(a,1:3), F.actors(a,4:5), clsColor(F.actorCls{a}), 0.85);
    end
    % tracks
    if ~isempty(F.tracks)
        plot(ax, F.tracks(:,1), F.tracks(:,2), 'ko','MarkerSize',7,'LineWidth',1.1);
    end
    % ego
    ec = [F.ego(1) + (cfg.veh.length/2 - cfg.veh.rearOH)*cos(F.ego(3)), ...
          F.ego(2) + (cfg.veh.length/2 - cfg.veh.rearOH)*sin(F.ego(3)), F.ego(3)];
    drawRect(ax, ec, [cfg.veh.length cfg.veh.width], [0.1 0.3 0.9], 1.0);

    xlim(ax, F.ego(1) + [-25 45]);
    ylim(ax, F.ego(2) + [-30 30]);
    title(ax, sprintf('%s   t=%5.2f s   v=%4.1f m/s   state=%s   vTgt=%4.1f', ...
        res.name, F.t, F.ego(4), F.state, F.vTarget), 'FontWeight','normal');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); grid(ax,'on');
    colormap(ax, hot);
    drawnow limitrate;
    writeVideo(vw, getframe(fig));
end
close(vw); close(fig);
end

% =======================================================================
function f = makePlots(res, cfg)
L = res.log;
f = fullfile(cfg.io.outDir, sprintf('%s_metrics.png', res.name));
fig = figure('Visible','off','Position',[50 50 1200 800],'Color','w');

subplot(3,2,1);
plot(L.t, L.v, 'LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('v [m/s]'); title('Speed');

subplot(3,2,2);
plot(L.latencyT, L.latency*1000, '.-'); grid on; hold on;
yline(cfg.metrics.latencyBudget*1000,'r--','budget');
xlabel('t [s]'); ylabel('ms'); title('Replanning latency');

subplot(3,2,3);
plot(L.t, L.curv, 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('\kappa [1/m]'); title('Path curvature (smoothness)');

subplot(3,2,4);
plot(L.t, min(L.minClear,10), 'LineWidth',1.4); grid on; hold on;
yline(cfg.safe.minClear,'r--','min clearance');
xlabel('t [s]'); ylabel('m'); title('Clearance to nearest agent');

subplot(3,2,5);
plot(L.t, L.d, 'LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('d [m]'); title('Lateral offset from route');

subplot(3,2,6);
states = unique(L.state);
idx = zeros(size(L.state));
for i = 1:numel(L.state), idx(i) = find(strcmp(states, L.state{i})); end
plot(L.t, idx, 'LineWidth',1.6); grid on;
set(gca,'YTick',1:numel(states),'YTickLabel',states);
ylim([0.5 numel(states)+0.5]); xlabel('t [s]'); title('Behaviour state');

if exist('sgtitle','file')
    sgtitle(sprintf('%s', res.name), 'FontWeight','bold');
else
    set(gcf,'Name',res.name,'NumberTitle','off');   % pre-R2018b fallback
end
print(fig, f, '-dpng','-r110');
close(fig);
end

% =======================================================================
function drawRoads(ax, map)
for i = 1:numel(map.roads)
    R = map.roads(i);
    pS = [R.leftSh; flipud(R.rightSh)];
    pD = [R.leftEdge; flipud(R.rightEdge)];
    patch(ax, pS(:,1), pS(:,2), [0.88 0.86 0.80], 'EdgeColor','none');
    patch(ax, pD(:,1), pD(:,2), [0.72 0.72 0.72], 'EdgeColor','none');
    if R.marked
        plot(ax, R.center(:,1), R.center(:,2), '--', 'Color',[1 1 1], 'LineWidth',1);
    end
end
end

function drawRect(ax, pose, sz, col, alp)
c = cos(pose(3)); s = sin(pose(3));
L = sz(1)/2; W = sz(2)/2;
loc = [L W; L -W; -L -W; -L W];
V = [pose(1) + loc(:,1)*c - loc(:,2)*s, pose(2) + loc(:,1)*s + loc(:,2)*c];
patch(ax, V(:,1), V(:,2), col, 'FaceAlpha', alp, 'EdgeColor','k','LineWidth',0.6);
end

function c = clsColor(cls)
switch cls
    case 'car',          c = [0.25 0.45 0.85];
    case {'bus','truck'},c = [0.10 0.25 0.55];
    case 'autorickshaw', c = [0.95 0.60 0.10];
    case 'twowheeler',   c = [0.80 0.20 0.75];
    case 'bicycle',      c = [0.40 0.75 0.85];
    case 'pedestrian',   c = [0.90 0.15 0.15];
    case 'animal',       c = [0.45 0.30 0.15];
    case 'pushcart',     c = [0.55 0.55 0.55];
    otherwise,           c = [0.30 0.30 0.30];
end
end
