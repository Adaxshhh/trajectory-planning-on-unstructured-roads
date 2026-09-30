function varargout = iadpFusion(mode, varargin)
%IADPFUSION  Camera/LiDAR/radar fusion and multi-object tracking.
%
%   trk            = iadpFusion('init', cfg)
%   [trk, tracks]  = iadpFusion('update', trk, dets, dt, cfg)
%
%   Filter: EKF on a constant-turn-rate-and-velocity (CTRV) state
%           x = [px py v psi omega]'
%   CTRV is deliberately chosen over constant velocity because Indian
%   traffic turns constantly: rickshaws swerving, two-wheelers filtering,
%   cattle changing direction. Process noise is class-adaptive, so once a
%   track is classified as an animal it is given a far larger manoeuvre
%   envelope than a bus.
%
%   Classification fuses camera labels (semantic) with LiDAR extent
%   (geometric) and kinematic plausibility (speed), which recovers the
%   correct class even when the camera confuses a pushcart with a car.

switch lower(mode)
    case 'init'
        varargout{1} = initTracker(varargin{:});
    case 'update'
        [trk, tracks] = updateTracker(varargin{:});
        varargout{1} = trk; varargout{2} = tracks;
    otherwise
        error('iadp:fusion','Unknown mode %s', mode);
end
end

% =======================================================================
function trk = initTracker(cfg)
trk.nextId  = 1;
trk.tracks  = emptyTrack();
trk.classes = {'car','bus','truck','autorickshaw','twowheeler', ...
               'bicycle','pedestrian','animal','pushcart','unknown'};
trk.cfg = cfg.track;
end

function t = emptyTrack()
t = struct('id',{},'x',{},'P',{},'hits',{},'misses',{},'confirmed',{}, ...
           'cls',{},'score',{},'extent',{},'age',{},'lastSeen',{});
end

% =======================================================================
function [trk, out] = updateTracker(trk, dets, dt, cfg)

nC = numel(trk.classes);

%% ---- predict ----------------------------------------------------------
for i = 1:numel(trk.tracks)
    T = trk.tracks(i);
    p = iadpClassParams(T.cls);
    [T.x, F] = ctrvPredict(T.x, dt);
    Q = ctrvQ(dt, p);
    T.P = F*T.P*F.' + Q;
    T.age = T.age + dt;
    trk.tracks(i) = T;
end

%% ---- associate (greedy global nearest neighbour with gating) ----------
nT = numel(trk.tracks); nD = numel(dets);
assigned = zeros(1,nD);
if nT > 0 && nD > 0
    C = inf(nT, nD);
    for i = 1:nT
        H = [1 0 0 0 0; 0 1 0 0 0];
        for j = 1:nD
            R = pickR(dets(j), cfg);
            S = H*trk.tracks(i).P*H.' + R + dets(j).R;
            nu = dets(j).pos(:) - trk.tracks(i).x(1:2);
            d2 = nu.'*(S\nu);
            if d2 < cfg.track.gate
                C(i,j) = d2 + 0.5*log(max(det(S),1e-9));
            end
        end
    end
    used = false(1,nT);
    while true
        [v, idx] = min(C(:));
        if ~isfinite(v), break; end
        [i, j] = ind2sub(size(C), idx);
        trk.tracks(i) = measUpdate(trk.tracks(i), dets(j), cfg, trk.classes);
        used(i) = true; assigned(j) = i;
        C(i,:) = inf; C(:,j) = inf;
    end
    for i = 1:nT
        if ~used(i)
            trk.tracks(i).misses = trk.tracks(i).misses + 1;
        else
            trk.tracks(i).misses = 0;
            trk.tracks(i).hits   = trk.tracks(i).hits + 1;
            trk.tracks(i).lastSeen = 0;
        end
    end
end

%% ---- spawn new tracks -------------------------------------------------
for j = 1:nD
    if assigned(j) ~= 0, continue; end
    if strcmp(dets(j).src,'radar') && isnan(dets(j).rate), continue; end
    T.id = trk.nextId; trk.nextId = trk.nextId + 1;
    v0 = 0;
    if ~isnan(dets(j).rate), v0 = max(0, abs(dets(j).rate)); end
    T.x = [dets(j).pos(1); dets(j).pos(2); v0; 0; 0];
    T.P = cfg.track.P0;
    T.hits = 1; T.misses = 0; T.confirmed = false;
    T.score = zeros(1,nC);
    if ~isempty(dets(j).cls)
        T.score(strcmp(trk.classes, dets(j).cls)) = dets(j).conf;
    end
    T.cls = 'unknown';
    T.extent = dets(j).extent;
    T.age = 0; T.lastSeen = 0;
    trk.tracks(end+1) = orderFields(T); %#ok<AGROW>
end

%% ---- confirm / prune --------------------------------------------------
keep = true(1,numel(trk.tracks));
for i = 1:numel(trk.tracks)
    T = trk.tracks(i);
    if T.hits >= cfg.track.confirmHits, T.confirmed = true; end
    if T.misses > cfg.track.maxMisses,  keep(i) = false; end
    T.cls = classify(T, trk.classes);
    T.x(3) = max(0, min(T.x(3), iadpClassParams(T.cls).vmax*1.2));
    T.x(5) = max(min(T.x(5), 3.5), -3.5);
    trk.tracks(i) = T;
end
trk.tracks = trk.tracks(keep);

%% ---- output confirmed tracks -----------------------------------------
out = trk.tracks([trk.tracks.confirmed]);
end

% =======================================================================
function T = measUpdate(T, d, cfg, classes)
H = [1 0 0 0 0; 0 1 0 0 0];
R = pickR(d, cfg) + d.R;
S = H*T.P*H.' + R;
K = (T.P*H.')/S;
nu= d.pos(:) - T.x(1:2);
T.x = T.x + K*nu;
T.P = (eye(5) - K*H)*T.P;

% radar range-rate as a scalar pseudo-measurement on speed
if ~isnan(d.rate) && T.x(3) > 0.2
    T.x(3) = 0.85*T.x(3) + 0.15*abs(d.rate);
end

% class evidence
if ~isempty(d.cls)
    T.score(strcmp(classes, d.cls)) = T.score(strcmp(classes, d.cls)) + d.conf;
end
if ~any(isnan(d.extent))
    T.extent = 0.7*orDefault(T.extent, d.extent) + 0.3*d.extent;
    for c = 1:numel(classes)
        p = iadpClassParams(classes{c});
        e = exp(-0.5*((p.L-T.extent(1))/0.9)^2 - 0.5*((p.W-T.extent(2))/0.5)^2);
        T.score(c) = T.score(c) + 0.35*e;
    end
end
T.P = 0.5*(T.P + T.P.');
end

function e = orDefault(e, alt)
if any(isnan(e)), e = alt; end
end

function R = pickR(d, cfg)
switch d.src
    case 'cam',   R = cfg.track.R_cam;
    case 'lidar', R = cfg.track.R_lidar;
    otherwise,    R = cfg.track.R_radar;
end
end

% =======================================================================
function cls = classify(T, classes)
sc = T.score;
v  = T.x(3);
% kinematic plausibility gate
for c = 1:numel(classes)
    p = iadpClassParams(classes{c});
    if v > p.vmax*1.25, sc(c) = sc(c) - 3.0; end
end
[m, i] = max(sc);
if m <= 0.2
    cls = 'unknown';
else
    cls = classes{i};
end
end

% =======================================================================
function [xn, F] = ctrvPredict(x, dt)
px = x(1); py = x(2); v = x(3); psi = x(4); w = x(5);
if abs(w) > 1e-4
    xn = [px + v/w*( sin(psi+w*dt) - sin(psi));
          py + v/w*(-cos(psi+w*dt) + cos(psi));
          v; psi + w*dt; w];
else
    xn = [px + v*cos(psi)*dt; py + v*sin(psi)*dt; v; psi + w*dt; w];
end
xn(4) = mod(xn(4)+pi, 2*pi) - pi;

if nargout < 2, F = []; return; end   % <- stops the recursion below

% numerical Jacobian (robust and cheap for a 5-state model)
F = eye(5); eps0 = 1e-6;
for k = 1:5
    dx = zeros(5,1); dx(k) = eps0;
    xp  = ctrvPredict(x+dx, dt);      % single output -> no further recursion
    dxn = xp - xn;
    dxn(4) = mod(dxn(4)+pi, 2*pi) - pi;   % heading wrap-safe
    F(:,k) = dxn/eps0;
end
end

function Q = ctrvQ(dt, p)
sa = p.amax*0.6*p.erratic;     % longitudinal manoeuvre noise
sw = p.yawmax*0.5*p.erratic;   % yaw-rate noise
G  = [0.5*dt^2 0; 0 0; dt 0; 0 0.5*dt^2; 0 dt];
Q  = G*diag([sa^2 sw^2])*G.';
Q  = Q + diag([1e-3 1e-3 1e-3 1e-4 1e-4]);
end

function T = orderFields(T)
T = orderfields(T, {'id','x','P','hits','misses','confirmed','cls', ...
                    'score','extent','age','lastSeen'});
end
