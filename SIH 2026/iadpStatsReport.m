function txt = iadpStatsReport(res, M, cfg)
%IADPSTATSREPORT  Detailed detect-and-dodge statistics for one run.
%Prints to the command window and writes results/<name>_stats.txt.
L = res.log;  dt = cfg.dt;  n = numel(L.t);
c = {};
c{end+1} = sprintf('=== %s : detect-and-dodge report ===', res.name);
c{end+1} = sprintf('Result            : %s', tern(M.pass,'PASS','FAIL'));
if ~M.pass, c{end+1} = sprintf('Fail reasons      : %s', strjoin(M.failReasons,'; ')); end
c{end+1} = sprintf('Goal reached      : %s (t = %.1f s, sim length %.1f s)', tern(res.goalReached,'yes','no'), tern(res.goalReached,res.goalTime,res.tEnd), res.tEnd);
c{end+1} = sprintf('Distance driven   : %.1f m of %.1f m route (%.0f %%)', M.distance, res.goal.s, 100*M.distance/max(res.goal.s,1));
c{end+1} = sprintf('Speed mean / max  : %.1f / %.1f m/s', M.meanSpeed, M.maxSpeed);
c{end+1} = '--- perception ---';
c{end+1} = sprintf('Detections / cycle: %.1f   confirmed tracks: %.1f   true actors: %d', mean(L.nDets(L.nDets>=0)), M.meanTracks, numel(res.actors));
c{end+1} = sprintf('Class accuracy    : %.0f %%', 100*M.classAccuracy);
c{end+1} = '--- dodging ---';
c{end+1} = sprintf('Collision         : %s', tern(res.collision, sprintf('YES with %s at t=%.1f s',res.collisionInfo.with,res.collisionInfo.t),'none'));
c{end+1} = sprintf('Min clearance     : %.2f m   min TTC: %.1f s   near-miss time (<%.1f m): %.1f s', M.minClearance, M.minTTC, cfg.safe.minClear, M.nearMisses);
c{end+1} = sprintf('Shoulder use      : %.0f %% of run   max lateral accel: %.2f m/s^2', 100*M.shoulderUse, M.maxLatAcc);
% per-actor closest approach (exact rectangle distance, every 4th step)
ego = struct('x',0,'y',0,'psi',0);
for i = 1:numel(res.actors)
    a = res.actors(i); dmin = inf; tmin = NaN;
    for k = 1:4:n
        ep = [L.x(k)+(cfg.veh.length/2-cfg.veh.rearOH)*cos(L.psi(k)), L.y(k)+(cfg.veh.length/2-cfg.veh.rearOH)*sin(L.psi(k)), L.psi(k)];
        if isnan(L.ax(k,i)), continue; end
        d = iadpRectDist(ep,[cfg.veh.length cfg.veh.width],[L.ax(k,i) L.ay(k,i) L.apsi(k,i)],[a.p.L a.p.W]);
        if d < dmin, dmin = d; tmin = L.t(k); end
    end
    c{end+1} = sprintf('  closest to %-13s #%d : %5.2f m at t=%.1f s', a.cls, a.id, dmin, tmin); %#ok<AGROW>
end
c{end+1} = '--- planner ---';
c{end+1} = sprintf('Replans %d (mean %.1f ms, p95 %.1f ms, max %.1f ms, budget %.0f ms)', M.replanCount, 1e3*M.replanLatencyMean, 1e3*M.replanLatencyP95, 1e3*M.replanLatencyMax, 1e3*res.checks.maxLatencyMean);
c{end+1} = sprintf('Plan failures %d   hybrid A* %d calls, success %.0f %%   stall-recovery events %d', M.planFailures, M.astarCalls, 100*tern(isnan(M.astarSuccessRate),0,M.astarSuccessRate), L.unstickEvents);
c{end+1} = sprintf('Smoothness: curvature RMS %.3f 1/m, jerk RMS %.2f m/s^3', M.pathSmoothCurvRMS, M.pathSmoothJerkRMS);
st = unique(L.state); s = '';
for i = 1:numel(st), s = [s sprintf('%s %.0f%%  ', st{i}, 100*mean(strcmp(L.state,st{i})))]; end %#ok<AGROW>
c{end+1} = ['Behaviour time   : ' s];
txt = strjoin(c, newline);
disp(txt);
if ~exist(cfg.io.outDir,'dir'), mkdir(cfg.io.outDir); end
fid = fopen(fullfile(cfg.io.outDir,[res.name '_stats.txt']),'w'); fprintf(fid,'%s\n',txt); fclose(fid);
end
function s = tern(cnd,a,b), if cnd, s = a; else, s = b; end, end
