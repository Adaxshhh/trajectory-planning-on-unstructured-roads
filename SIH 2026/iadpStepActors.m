function actors = iadpStepActors(actors, dt, t, ego, map)
%IADPSTEPACTORS  Advance every ground-truth agent by dt.
%
%   Implements the behaviours that make Indian traffic hard: unsignalled
%   cut-ins, weaving without lane discipline, pedestrians crossing at
%   unmarked points, cattle wandering, and carts that simply stop.

for k = 1:numel(actors)
    a = actors(k);
    if ~a.alive, actors(k) = a; continue; end
    a.t = t;
    vOld = a.v;  psiOld = a.psi;

    switch a.beh
        case 'constant'
            a.v = approach(a.v, a.vTarget, a.p.amax, dt);

        case {'follow','merge','rrTrack'}
            [a, steer] = trackWaypoints(a, dt);
            vt = a.vTarget;
            if strcmp(a.beh,'merge') && t >= a.trigT
                vt = a.vTarget*(1 + 0.35*a.aggr);      % squeeze into the gap
            end
            a.v   = approach(a.v, vt, a.p.amax, dt);
            a.psi = a.psi + steer*dt;

        case 'weave'
            % lateral sinusoid = no lane discipline
            a.psi = a.psi + (2*pi/a.per)*a.amp*cos(2*pi*t/a.per + a.phase)*dt;
            a.v   = approach(a.v, a.vTarget, a.p.amax, dt);

        case 'cutin'
            if t >= a.trigT
                dy  = (ego.y - a.y);  dx = (ego.x - a.x);
                tgt = atan2(dy, dx);
                a.psi = a.psi + wrapAngle(tgt - a.psi)*min(1, 1.2*a.aggr)*dt;
            end
            a.v = approach(a.v, a.vTarget, a.p.amax, dt);

        case 'cross'
            if t >= a.trigT
                a.v = approach(a.v, a.vTarget, a.p.amax, dt);
            else
                a.v = approach(a.v, 0, a.p.amax, dt);
            end

        case 'stopped'
            a.v = 0;

        case 'wander'
            % cattle: slow random walk with long correlation time
            a.omega = 0.92*a.omega + 0.35*randn*a.p.yawmax*dt/0.05;
            a.omega = max(min(a.omega, a.p.yawmax), -a.p.yawmax);
            a.psi   = a.psi + a.omega*dt;
            if t >= a.trigT
                a.v = approach(a.v, a.vTarget, a.p.amax, dt);
            else
                a.v = approach(a.v, 0.2, a.p.amax, dt);
            end

        otherwise
            a.v = approach(a.v, a.vTarget, a.p.amax, dt);
    end

    % simple mutual avoidance so agents do not drive through each other
    a = repelFromEgo(a, ego, dt);

    a.v = max(0, min(a.v, a.p.vmax));
    a.x = a.x + a.v*cos(a.psi)*dt;
    a.y = a.y + a.v*sin(a.psi)*dt;
    if dt > 0
        a.omega = wrapAngle(a.psi - psiOld)/dt;
        a.acc   = (a.v - vOld)/dt;
    end

    % retire agents that leave the map
    if nargin >= 5 && ~isempty(map)
        if a.x < map.bbox(1)-20 || a.x > map.bbox(3)+20 || ...
           a.y < map.bbox(2)-20 || a.y > map.bbox(4)+20
            a.alive = false;
        end
    end
    actors(k) = a;
end
end

% -----------------------------------------------------------------------
function [a, steer] = trackWaypoints(a, dt)
steer = 0;
if isempty(a.wp), return; end
n = size(a.wp,1);
i = min(a.wpIdx, n);
d = hypot(a.wp(i,1)-a.x, a.wp(i,2)-a.y);
while d < max(2.0, 0.6*a.v) && i < n
    i = i + 1;
    d = hypot(a.wp(i,1)-a.x, a.wp(i,2)-a.y);
end
a.wpIdx = i;
tgt   = atan2(a.wp(i,2)-a.y, a.wp(i,1)-a.x);
err   = wrapAngle(tgt - a.psi);
steer = max(min(1.8*err, a.p.yawmax), -a.p.yawmax);
if i == n && d < 2.0, a.vTarget = min(a.vTarget, 0); end
end

function a = repelFromEgo(a, ego, dt)
d = hypot(ego.x-a.x, ego.y-a.y);
if d < 4.0 && a.p.vru
    % VRUs do flinch away from a car that is very close
    away = atan2(a.y-ego.y, a.x-ego.x);
    a.psi = a.psi + wrapAngle(away - a.psi)*0.8*dt;
end
end

function v = approach(v, vt, amax, dt)
dv = vt - v;
v  = v + max(min(dv, amax*dt), -1.5*amax*dt);
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end
