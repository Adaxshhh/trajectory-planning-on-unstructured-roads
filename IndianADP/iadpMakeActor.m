function a = iadpMakeActor(id, cls, x, y, psi, v, beh, aggr, varargin)
%IADPMAKEACTOR  Ground-truth traffic participant.
%
%   a = iadpMakeActor(id, class, x, y, psi, v, behavior, aggression, ...)
%
%   behavior:
%     'constant'   keep speed and heading
%     'follow'     follow the waypoint list in a.wp
%     'weave'      lateral sinusoidal drift (no lane discipline)
%     'cutin'      drift toward the ego lane after t = a.trigT
%     'cross'      walk/run across the road, starts at a.trigT
%     'merge'      accelerate along wp and merge without signalling
%     'stopped'    parked / broken down (pushcart, bus at stop)
%     'wander'     random walk (cattle, stray dogs)
%     'rrTrack'    replay of a RoadRunner trajectory in a.wp
%
%   Optional name/value: 'wp',[N x 2], 'trigT',t, 'vTarget',v, 'amp',m, 'per',s

p = iadpClassParams(cls);
a.id = id;  a.cls = p.cls;  a.p = p;
a.x = x;  a.y = y;  a.psi = psi;  a.v = v;
a.beh = beh;  a.aggr = aggr;
a.wp = [];  a.wpIdx = 1;  a.trigT = 0;  a.vTarget = v;
a.amp = 0.8;  a.per = 5;  a.alive = true;  a.t = 0;  a.phase = rand*2*pi;
a.omega = 0;  a.acc = 0;

for k = 1:2:numel(varargin)
    a.(varargin{k}) = varargin{k+1};
end
end
