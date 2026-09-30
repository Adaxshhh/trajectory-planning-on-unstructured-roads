function [delta, aCmd, ctl] = iadpController(ego, traj, cfg, ctl, dt)
%IADPCONTROLLER  Track the planned trajectory.
%
%   [delta, aCmd, ctl] = iadpController(ego, traj, cfg, ctl, dt)
%
%   Lateral      : pure pursuit with speed-scheduled lookahead
%   Longitudinal : PI on the trajectory speed at the current arc length,
%                  with feed-forward from the trajectory acceleration
%
%   ctl.eInt  integral state (initialise with ctl = struct('eInt',0))

if isempty(ctl) || ~isfield(ctl,'eInt'), ctl.eInt = 0; end
if isempty(traj) || numel(traj.x) < 2
    delta = 0; aCmd = cfg.veh.aMin; return;
end

%% ---- lateral: pure pursuit -------------------------------------------
Ld = min(max(cfg.ctrl.LdGain*ego.v, cfg.ctrl.LdMin), cfg.ctrl.LdMax);

d  = hypot(traj.x - ego.x, traj.y - ego.y);
[~, iNear] = min(d);
iTgt = iNear;
while iTgt < numel(d) && d(iTgt) < Ld
    iTgt = iTgt + 1;
end

dx = traj.x(iTgt) - ego.x;
dy = traj.y(iTgt) - ego.y;
alpha = wrapA(atan2(dy,dx) - ego.psi);
LdAct = max(hypot(dx,dy), 0.5);

delta = atan2(2*cfg.veh.L*sin(alpha), LdAct);

% cross-track correction at very low speed (pure pursuit degenerates)
if ego.v < 1.5
    n  = [-sin(ego.psi) cos(ego.psi)];
    e  = ([traj.x(iNear) traj.y(iNear)] - [ego.x ego.y])*n.';
    delta = delta + 0.25*e;
end
delta = max(min(delta, cfg.veh.maxSteer), -cfg.veh.maxSteer);

%% ---- longitudinal: PI + feed-forward ---------------------------------
vRef = traj.v(iNear);
if iNear < numel(traj.v)
    dtRef = max(traj.t(iNear+1) - traj.t(iNear), 1e-3);
    aFF   = (traj.v(iNear+1) - traj.v(iNear))/dtRef;
else
    aFF = 0;
end
e = vRef - ego.v;
ctl.eInt = max(min(ctl.eInt + e*dt, cfg.ctrl.iMax), -cfg.ctrl.iMax);
aCmd = aFF + cfg.ctrl.Kp*e + cfg.ctrl.Ki*ctl.eInt;

if vRef < 0.05
    aCmd = min(aCmd, -1.5);         % commit to the stop
    ctl.eInt = 0;
end
aCmd = max(min(aCmd, cfg.veh.aMax), cfg.veh.aMin);

ctl.iNear = iNear; ctl.iTgt = iTgt; ctl.vRef = vRef; ctl.Ld = Ld;
end

function a = wrapA(a)
a = mod(a + pi, 2*pi) - pi;
end
