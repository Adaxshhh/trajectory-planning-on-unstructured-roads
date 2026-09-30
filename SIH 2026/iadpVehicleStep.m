function ego = iadpVehicleStep(ego, deltaCmd, aCmd, cfg, dt)
%IADPVEHICLESTEP  Advance the ego vehicle one integration step.
%
%   ego = iadpVehicleStep(ego, deltaCmd, aCmd, cfg, dt)
%
%   ego.x, ego.y   rear-axle position [m]
%   ego.psi        yaw [rad]
%   ego.v          longitudinal speed [m/s]
%   ego.delta      actual road-wheel angle [rad]
%   ego.a          actual longitudinal acceleration [m/s^2]
%   ego.vy, ego.r  lateral velocity / yaw rate (dynamic model only)
%
%   cfg.veh.model = 'kinematic'  -> RK4 bicycle model (Simulink-equivalent)
%                 = 'dynamic'    -> linear single-track with tyre slip
%
%   Actuator realism matters here: steering-rate and jerk limits are what
%   stop a planner from "cheating" its way out of a cattle-crossing event.

if ~isfield(ego,'vy'), ego.vy = 0; end
if ~isfield(ego,'r'),  ego.r  = 0; end
if ~isfield(ego,'a'),  ego.a  = 0; end
if ~isfield(ego,'delta'), ego.delta = 0; end

% ---- actuator limits ---------------------------------------------------
deltaCmd = max(min(deltaCmd, cfg.veh.maxSteer), -cfg.veh.maxSteer);
dDelta   = max(min(deltaCmd - ego.delta, cfg.veh.maxSteerRt*dt), -cfg.veh.maxSteerRt*dt);
ego.delta= ego.delta + dDelta;

aCmd = max(min(aCmd, cfg.veh.aMax), cfg.veh.aMin);
dA   = max(min(aCmd - ego.a, cfg.veh.jerkMax*dt), -cfg.veh.jerkMax*dt);
ego.a= ego.a + dA;

switch lower(cfg.veh.model)
    case 'dynamic'
        ego = stepDynamic(ego, cfg, dt);
    otherwise
        ego = stepKinematic(ego, cfg, dt);
end

ego.v   = max(0, ego.v);
ego.psi = mod(ego.psi + pi, 2*pi) - pi;
end

% =======================================================================
function ego = stepKinematic(ego, cfg, dt)
z = [ego.x; ego.y; ego.psi; ego.v];
f = @(z) [z(4)*cos(z(3)); z(4)*sin(z(3)); z(4)*tan(ego.delta)/cfg.veh.L; ego.a];
k1 = f(z);
k2 = f(z + dt/2*k1);
k3 = f(z + dt/2*k2);
k4 = f(z + dt*k3);
z  = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);
ego.x = z(1); ego.y = z(2); ego.psi = z(3); ego.v = max(0,z(4));
ego.r  = ego.v*tan(ego.delta)/cfg.veh.L;
ego.vy = 0;
end

% =======================================================================
function ego = stepDynamic(ego, cfg, dt)
vx = max(ego.v, 0.1);
if vx < 2.0                      % low speed: tyre model is ill-conditioned
    ego = stepKinematic(ego, cfg, dt); return;
end
m = cfg.veh.m; Iz = cfg.veh.Iz;
lf= cfg.veh.lf; lr = cfg.veh.lr;
Cf= cfg.veh.Cf; Cr = cfg.veh.Cr;
d = ego.delta;

nSub = 4; h = dt/nSub;
vy = ego.vy; r = ego.r; psi = ego.psi; x = ego.x; y = ego.y;
for k = 1:nSub
    af = d - (vy + lf*r)/vx;
    ar =   - (vy - lr*r)/vx;
    Fyf= 2*Cf*af;  Fyr = 2*Cr*ar;
    vyd= (Fyf*cos(d) + Fyr)/m - vx*r;
    rd = (lf*Fyf*cos(d) - lr*Fyr)/Iz;
    vy = vy + h*vyd;
    r  = r  + h*rd;
    psi= psi + h*r;
    x  = x + h*(vx*cos(psi) - vy*sin(psi));
    y  = y + h*(vx*sin(psi) + vy*cos(psi));
    vx = max(0.1, vx + h*ego.a);
end
ego.vy = vy; ego.r = r; ego.psi = psi; ego.x = x; ego.y = y; ego.v = vx;
end
