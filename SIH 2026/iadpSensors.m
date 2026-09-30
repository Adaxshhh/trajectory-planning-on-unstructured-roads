function dets = iadpSensors(cfg, ego, actors, t)
%IADPSENSORS  Multi-sensor detection generation (camera + LiDAR + radar).
%
%   dets = iadpSensors(cfg, ego, actors, t)
%
%   Each detection:
%     .pos     [1x2] measured position in WORLD frame
%     .R       [2x2] measurement covariance in WORLD frame
%     .src     'cam' | 'lidar' | 'radar'
%     .cls     class label ('' if the sensor cannot classify)
%     .conf    classification confidence
%     .extent  [L W] (LiDAR only, NaN otherwise)
%     .rate    range-rate [m/s] (radar only, NaN otherwise)
%     .truthId ground-truth id (-1 for clutter; diagnostics only)
%
%   Modelled effects: limited range/FOV, per-sensor accuracy, probability of
%   detection that decays with range, mutual occlusion, class confusion for
%   the camera, and false alarms.

dets = emptyDet();
sens = {'cam','lidar','radar'};

for si = 1:numel(sens)
    sn  = sens{si};
    cs  = cfg.sensor.(sn);
    org = ego2world(ego, cs.mountXY);

    for k = 1:numel(actors)
        a = actors(k);
        if ~a.alive, continue; end
        dx = a.x - org(1); dy = a.y - org(2);
        rng_ = hypot(dx,dy);
        if rng_ > cs.range || rng_ < 0.3, continue; end
        az = wrapAngle(atan2(dy,dx) - ego.psi);
        if abs(az) > cs.fov/2, continue; end
        if cfg.sensor.occlusionOn && isOccluded(org, a, actors, k), continue; end

        pd = cs.pd * (1 - 0.35*(rng_/cs.range)^2);
        if strcmp(sn,'cam') && a.p.r < 0.6, pd = pd*0.9; end   % small VRUs
        if rand > pd, continue; end

        [zr, zaz] = deal(rng_ + cs.sigRange*randn, az + cs.sigAz*randn);
        px = org(1) + zr*cos(zaz + ego.psi);
        py = org(2) + zr*sin(zaz + ego.psi);

        R = polarToCartCov(zr, zaz + ego.psi, cs.sigRange, cs.sigAz);

        d = newDet();
        d.pos = [px py];  d.R = R;  d.src = sn;  d.truthId = a.id;
        d.cls = '';  d.conf = 0;  d.extent = [NaN NaN];  d.rate = NaN;

        switch sn
            case 'cam'
                d.cls  = confuseClass(a.cls, cs.classAcc, rng_/cs.range);
                d.conf = max(0.35, cs.classAcc - 0.3*rng_/cs.range);
            case 'lidar'
                d.extent = [a.p.L a.p.W] + cs.sigExtent*randn(1,2);
                d.extent = max(d.extent, 0.2);
            case 'radar'
                vr = [a.v*cos(a.psi) a.v*sin(a.psi)] - [ego.v*cos(ego.psi) ego.v*sin(ego.psi)];
                los = [cos(zaz+ego.psi) sin(zaz+ego.psi)];
                d.rate = vr*los.' + cs.sigRate*randn;
        end
        dets(end+1) = d; %#ok<AGROW>
    end

    % ---- clutter -------------------------------------------------------
    nFa = poissrndLocal(cfg.sensor.clutterRate/numel(sens));
    for f = 1:nFa
        rr = 5 + (cs.range-5)*rand;
        aa = (rand-0.5)*cs.fov;
        d = newDet();
        d.pos = org + rr*[cos(aa+ego.psi) sin(aa+ego.psi)];
        d.R = polarToCartCov(rr, aa+ego.psi, cs.sigRange*2, cs.sigAz*2);
        d.src = sn; d.cls = ''; d.conf = 0;
        d.extent = [NaN NaN]; d.rate = NaN; d.truthId = -1;
        dets(end+1) = d; %#ok<AGROW>
    end
end
end

% =======================================================================
function d = emptyDet()
%EMPTYDET  1x0 struct array template (field order is fixed).
d = struct('pos',{},'R',{},'src',{},'cls',{},'conf',{}, ...
           'extent',{},'rate',{},'truthId',{});
end

function d = newDet()
d = struct('pos',[0 0],'R',eye(2),'src','','cls','','conf',0, ...
           'extent',[NaN NaN],'rate',NaN,'truthId',-1);
end

function p = ego2world(ego, mount)
p = [ego.x + mount(1)*cos(ego.psi) - mount(2)*sin(ego.psi), ...
     ego.y + mount(1)*sin(ego.psi) + mount(2)*cos(ego.psi)];
end

function R = polarToCartCov(r, th, sr, sa)
J = [cos(th) -r*sin(th); sin(th) r*cos(th)];
R = J*diag([sr^2 sa^2])*J.';
R = R + 1e-4*eye(2);
end

function tf = isOccluded(org, a, actors, selfIdx)
tf = false;
pa = [a.x a.y];
d  = pa - org;  L = hypot(d(1),d(2));
if L < 1e-6, return; end
u = d/L;
for j = 1:numel(actors)
    if j == selfIdx, continue; end
    b = actors(j);
    if ~b.alive, continue; end
    pb = [b.x b.y] - org;
    proj = pb*u.';
    if proj <= 0.5 || proj >= L - 0.5, continue; end
    lat = abs(pb*[-u(2); u(1)]);
    if lat < 0.65*b.p.r && b.p.r > 0.8*a.p.r
        tf = true; return;
    end
end
end

function c = confuseClass(trueCls, acc, rngFrac)
% Camera class confusion, weighted toward visually similar Indian classes.
if rand < acc - 0.25*rngFrac
    c = trueCls; return;
end
groups = { {'car','autorickshaw','pushcart'}, ...
           {'twowheeler','bicycle','pedestrian'}, ...
           {'bus','truck'}, ...
           {'animal','pedestrian','pushcart'} };
for g = 1:numel(groups)
    if any(strcmp(trueCls, groups{g}))
        cand = setdiff(groups{g}, trueCls);
        c = cand{randi(numel(cand))};
        return;
    end
end
c = 'unknown';
end

function n = poissrndLocal(lam)
% Knuth, so no Statistics Toolbox dependency.
L = exp(-lam); k = 0; p = 1;
while true
    k = k + 1; p = p*rand;
    if p <= L, n = k-1; return; end
    if k > 50, n = 50; return; end
end
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end
