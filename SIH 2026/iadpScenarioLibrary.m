function spec = iadpScenarioLibrary(cfg, name)
%IADPSCENARIOLIBRARY  The five mandatory Indian validation scenarios.
%
%   spec = iadpScenarioLibrary(cfg)          % returns a cell of all names
%   spec = iadpScenarioLibrary(cfg, name)    % returns one scenario spec
%
%   Scenarios
%     1 VillageRoad        unmarked village road, unclear edges, oncoming bus
%     2 UrbanIntersection  busy 4-way junction with NO signals
%     3 HighwayMerge       merging slow-moving truck from an on-ramp
%     4 MarketDense        dense market, mixed traffic, crowd
%     5 CattleCrossing     sudden cattle crossing at speed
%
%   Each spec carries:
%     .scene .scenarioFile .routeIdx .startXY .egoStartS .egoV0 .goalS
%     .tMax .desc .actorFcn(map,cfg)   .checks  (scenario pass criteria)

allNames = {'VillageRoad','UrbanIntersection','HighwayMerge', ...
            'MarketDense','CattleCrossing'};
if nargin < 2 || isempty(name)
    spec = allNames; return;
end

s = struct();
s.name         = name;
s.scenarioFile = name;
s.tMax         = cfg.tMax;
s.egoV0        = 6;
s.goalS        = inf;                 % clamped to route length by the loader
s.startXY      = [];
s.egoStartS    = 5;
s.checks       = defaultChecks();

switch name
%% ======================================================================
case 'VillageRoad'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 7;
    s.tMax      = 45;
    s.desc      = ['Narrow unmarked village road. A pushcart is parked in ' ...
                   'the travel lane, an oncoming bus occupies the far side, ' ...
                   'a pedestrian steps out at an unmarked point and a goat ' ...
                   'wanders near the soft edge. The ego must use the ' ...
                   'shoulder to pass and give way to the bus.'];
    s.actorFcn  = @actorsVillage;
    s.checks.minClearance = 0.45;
    s.checks.mustUseShoulder = true;

%% ======================================================================
case 'UrbanIntersection'
    s.scene     = 'IndianUrbanIntersection';
    s.routeIdx  = [1 5 2];
    s.startXY   = [-78 0];
    s.egoV0     = 8;
    s.tMax      = 45;
    s.desc      = ['Unsignalised urban crossroads. Cross traffic enters ' ...
                   'without stopping, an auto-rickshaw cuts in from the ' ...
                   'left, pedestrians cross the box informally. The ego ' ...
                   'must negotiate the junction by yielding, not by rule.'];
    s.actorFcn  = @actorsIntersection;
    s.checks.mustYield = true;

%% ======================================================================
case 'HighwayMerge'
    s.scene     = 'IndianHighwayMerge';
    s.routeIdx  = 1;
    s.startXY   = [-50 0];
    s.egoStartS = 10;
    s.egoV0     = 19;
    s.tMax      = 40;
    s.desc      = ['Highway with a slow overloaded truck merging from the ' ...
                   'on-ramp without signalling, plus a slow tractor ahead ' ...
                   'in the running lane. The ego must either create a gap ' ...
                   'or overtake smoothly.'];
    s.actorFcn  = @actorsHighway;
    s.checks.maxJerkRMS = 3.5;

%% ======================================================================
case 'MarketDense'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 4;
    s.tMax      = 60;
    s.desc      = ['Dense market street: pedestrians on the carriageway, ' ...
                   'parked pushcarts, weaving two-wheelers and an ' ...
                   'auto-rickshaw stopping without warning. Expected ' ...
                   'behaviour is sustained low-speed creeping, not stopping.'];
    s.actorFcn  = @actorsMarket;
    s.checks.maxSpeed      = 6.0;
    s.checks.mustCreep     = true;
    s.checks.minClearance  = 0.40;

%% ======================================================================
case 'CattleCrossing'
    s.scene     = 'IndianVillageRoad';
    s.routeIdx  = 1;
    s.startXY   = [0 0];
    s.egoV0     = 11;
    s.tMax      = 40;
    s.desc      = ['Cattle emerge suddenly from the left shoulder at ' ...
                   'cruising speed. The ego must detect, predict the ' ...
                   'crossing intent, brake and/or swerve, hold while the ' ...
                   'herd clears, then resume.'];
    s.actorFcn  = @actorsCattle;
    s.checks.mustStopOrSwerve = true;
    s.checks.minClearance     = 0.50;

otherwise
    error('iadp:scenario','Unknown scenario "%s". Valid: %s', name, strjoin(allNames,', '));
end

s.allNames = allNames;
spec = s;
end

% =======================================================================
function c = defaultChecks()
c.noCollision        = true;
c.mustReachGoal      = true;
c.maxLatencyMean     = 0.100;   % [s]
c.minClearance       = 0.50;    % [m]
c.maxJerkRMS         = 5.0;     % [m/s^3]
c.maxCurvRMS         = 0.20;    % [1/m]
c.mustUseShoulder    = false;
c.mustYield          = false;
c.mustCreep          = false;
c.mustStopOrSwerve   = false;
c.maxSpeed           = inf;
end

% =======================================================================
% Actor sets. Each receives the built map so positions can be expressed
% along the route, which keeps them valid for any scene geometry.
% =======================================================================
function A = actorsVillage(map, cfg) %#ok<INUSD>
P = map.route;  A = [];
% 1 - pushcart abandoned in the travel lane
[x,y,h] = atS(P, 55, 0.2);
A = [A iadpMakeActor(1,'pushcart',x,y,h,0,'stopped',0.0)];
% 2 - oncoming bus on the far side (drives against the route direction)
[x,y,h] = atS(P, 150, -3.0);
wp = routeWP(P, 150, 20, -3.0, true);
A = [A iadpMakeActor(2,'bus',x,y,h+pi,9,'follow',0.4,'wp',wp,'vTarget',9)];
% 3 - two-wheeler weaving up behind the ego, no lane discipline
[x,y,h] = atS(P, 22, -0.8);
A = [A iadpMakeActor(3,'twowheeler',x,y,h,9,'weave',0.8,'vTarget',10,'amp',0.55,'per',4)];
% 4 - pedestrian crossing at an unmarked point
[x,y,h] = atS(P, 92, 4.2);
A = [A iadpMakeActor(4,'pedestrian',x,y,h-pi/2,0,'cross',0.6,'vTarget',1.4,'trigT',5.5)];
% 5 - goat near the fuzzy road edge
[x,y,h] = atS(P, 135, 3.4);
A = [A iadpMakeActor(5,'animal',x,y,h,0.3,'wander',0.5,'vTarget',1.0,'trigT',2)];
% 6 - auto-rickshaw ahead, slow
[x,y,h] = atS(P, 75, -0.4);
A = [A iadpMakeActor(6,'autorickshaw',x,y,h,5,'weave',0.6,'vTarget',6,'amp',0.35,'per',6)];
end

% -----------------------------------------------------------------------
function A = actorsIntersection(map, cfg) %#ok<INUSD>
P = map.route;  A = [];
% cross traffic south -> north straight through the box, does not stop
A = [A iadpMakeActor(1,'car', 3.0,-42, pi/2, 9,'follow',0.9, ...
        'wp',[3 -20; 3 0; 3 30; 3 70],'vTarget',9)];
A = [A iadpMakeActor(2,'twowheeler', 6.5,-60, pi/2, 12,'follow',0.9, ...
        'wp',[6.5 -25; 6 0; 5.5 30; 5.5 70],'vTarget',12)];
% cross traffic north -> south
A = [A iadpMakeActor(3,'bus', -3.0, 55, -pi/2, 7,'follow',0.5, ...
        'wp',[-3 20; -3 0; -3 -30; -3 -70],'vTarget',7)];
% auto-rickshaw cutting in from the left just before the box
[x,y,h] = atS(P, 48, 3.2);
A = [A iadpMakeActor(4,'autorickshaw',x,y,h,6,'cutin',0.85,'vTarget',7,'trigT',2.5)];
% pedestrians crossing the junction informally
A = [A iadpMakeActor(5,'pedestrian', -12, -6, pi/2, 0,'cross',0.6,'vTarget',1.3,'trigT',3.0)];
A = [A iadpMakeActor(6,'pedestrian', -10.5,-6.6,pi/2, 0,'cross',0.6,'vTarget',1.2,'trigT',3.4)];
% slow cyclist ahead on the exit road
A = [A iadpMakeActor(7,'bicycle', 26, 1.4, 0, 4,'weave',0.5,'vTarget',4.5,'amp',0.4,'per',5)];
end

% -----------------------------------------------------------------------
function A = actorsHighway(map, cfg) %#ok<INUSD>
P = map.route;  A = [];
% slow overloaded truck coming up the ramp and merging without signalling
A = [A iadpMakeActor(1,'truck', -22,-22, atan2(26,150), 12,'merge',0.9, ...
        'wp',[10 -14; 45 -6; 75 1.4; 140 1.4; 260 1.4], 'vTarget',13,'trigT',2)];
% slow tractor-like vehicle already in the running lane
[x,y,h] = atS(P, 95, 0.0);
A = [A iadpMakeActor(2,'truck',x,y,h,8,'constant',0.2,'vTarget',8)];
% fast car closing from behind in the outer lane
[x,y,h] = atS(P, 5, -3.4);
A = [A iadpMakeActor(3,'car',x,y,h,24,'constant',0.7,'vTarget',24)];
% two-wheeler filtering between lanes
[x,y,h] = atS(P, 60, -1.8);
A = [A iadpMakeActor(4,'twowheeler',x,y,h,17,'weave',0.9,'vTarget',18,'amp',0.5,'per',3.5)];
end

% -----------------------------------------------------------------------
function A = actorsMarket(map, cfg) %#ok<INUSD>
P = map.route;  A = [];
id = 0;
% parked / stopped carts on both edges
cartS = [28 46 64 88 112 132];
cartD = [ 2.6 -2.8 2.7 -2.6 2.8 -2.7];
for k = 1:numel(cartS)
    id = id + 1;
    [x,y,h] = atS(P, cartS(k), cartD(k));
    A = [A iadpMakeActor(id,'pushcart',x,y,h,0,'stopped',0)]; %#ok<AGROW>
end
% shoppers walking along and across the carriageway
pedS = [20 35 52 70 84 100 118 140];
for k = 1:numel(pedS)
    id = id + 1;
    side = (-1)^k;
    [x,y,h] = atS(P, pedS(k), 2.3*side);
    A = [A iadpMakeActor(id,'pedestrian',x,y,h-side*pi/2,0,'cross',0.5, ...
        'vTarget',1.1,'trigT',1.5+0.9*k)]; %#ok<AGROW>
end
% weaving two-wheelers and a stopping rickshaw
id = id+1; [x,y,h] = atS(P, 40, -1.0);
A = [A iadpMakeActor(id,'twowheeler',x,y,h,5,'weave',0.9,'vTarget',5.5,'amp',0.7,'per',3)];
id = id+1; [x,y,h] = atS(P, 66, 0.6);
A = [A iadpMakeActor(id,'autorickshaw',x,y,h,3,'stopped',0.4)];
id = id+1; [x,y,h] = atS(P, 105, -0.8);
A = [A iadpMakeActor(id,'bicycle',x,y,h,3.5,'weave',0.6,'vTarget',4,'amp',0.5,'per',4)];
id = id+1; [x,y,h] = atS(P, 150, -2.4);
wp = routeWP(P, 150, 25, -2.4, true);
A = [A iadpMakeActor(id,'autorickshaw',x,y,h+pi,4,'follow',0.5,'wp',wp,'vTarget',4)];
end

% -----------------------------------------------------------------------
function A = actorsCattle(map, cfg) %#ok<INUSD>
P = map.route;  A = [];
% a small herd stepping out from the left shoulder, no warning
base = 62;
for k = 1:3
    [x,y,h] = atS(P, base + 2.2*(k-1), 4.6 + 0.5*k);
    A = [A iadpMakeActor(k,'animal',x,y,h-pi/2,0.1,'cross',0.5, ...
        'vTarget',1.6+0.3*k,'trigT',3.2+0.35*k)]; %#ok<AGROW>
end
% a calf that then wanders back into the road (the classic second event)
[x,y,h] = atS(P, 70, 3.8);
A = [A iadpMakeActor(4,'animal',x,y,h,0.2,'wander',0.5,'vTarget',1.2,'trigT',5.5)];
% a vehicle following the ego, so braking has consequences
[x,y,h] = atS(P, 30, -0.3);
A = [A iadpMakeActor(5,'autorickshaw',x,y,h,10,'constant',0.6,'vTarget',10)];
end

% =======================================================================
% helpers
% =======================================================================
function [x,y,h] = atS(P, s, dOff)
%ATS  Point at arc length s with lateral offset dOff (+ = left).
s = min(max(s, 0), P.len);
i = max(1, min(numel(P.s), round(s/max(P.s(2)-P.s(1),1e-6)) + 1));
h = P.hdg(i);
n = [-sin(h) cos(h)];
x = P.xy(i,1) + n(1)*dOff;
y = P.xy(i,2) + n(2)*dOff;
end

function wp = routeWP(P, s0, n, dOff, reverse)
%ROUTEWP  Waypoint list along the route starting at s0.
step = 8;
ss = s0 + (0:n-1)*step*(1 - 2*reverse);
ss = ss(ss >= 0 & ss <= P.len);
wp = zeros(numel(ss),2);
for k = 1:numel(ss)
    [wp(k,1), wp(k,2)] = atS(P, ss(k), dOff);
end
if isempty(wp), wp = P.xy(end,:); end
end
