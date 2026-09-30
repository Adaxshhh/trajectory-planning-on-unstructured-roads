function p = iadpClassParams(cls)
%IADPCLASSPARAMS  Size / dynamics / predictability per Indian road-user class.
%
%   p = iadpClassParams('autorickshaw')
%
%   p.L, p.W     [m] bounding box
%   p.r          [m] collision radius used for fast disc checks
%   p.vmax       [m/s]
%   p.amax       [m/s^2] plausible longitudinal acceleration
%   p.yawmax     [rad/s] plausible turn rate  (rickshaws & bikes are high)
%   p.erratic    1 = textbook, 3 = completely unpredictable (animals)
%   p.vru        true for vulnerable road users (bigger safety buffer)
%   p.laneBound  true if the agent tends to respect lane geometry
%
%   Classes: car bus truck autorickshaw twowheeler bicycle pedestrian
%            animal pushcart unknown

switch lower(cls)
    case 'car'
        p = mk(4.3,1.8, 22, 3.0, 0.60, 1.0, false, true);
    case 'bus'
        p = mk(11.0,2.6, 16, 1.6, 0.35, 1.0, false, true);
    case 'truck'
        p = mk(8.5,2.5, 18, 1.4, 0.30, 1.1, false, true);
    case 'autorickshaw'
        p = mk(2.6,1.4, 13, 2.5, 1.40, 2.2, false, false);
    case 'twowheeler'
        p = mk(1.9,0.8, 20, 3.5, 1.80, 2.4, true,  false);
    case 'bicycle'
        p = mk(1.7,0.6,  8, 1.2, 1.50, 2.0, true,  false);
    case 'pedestrian'
        p = mk(0.6,0.6,  2.2, 1.5, 3.50, 2.8, true, false);
    case 'animal'
        p = mk(2.2,1.0,  4.5, 2.0, 2.50, 3.0, true, false);
    case 'pushcart'
        p = mk(2.4,1.2,  2.0, 0.8, 1.20, 1.8, true, false);
    otherwise   % unknown -> conservative envelope
        p = mk(4.0,2.0, 18, 3.0, 1.50, 2.5, true, false);
end
p.cls = lower(cls);
end

function p = mk(L,W,vmax,amax,yawmax,erratic,vru,laneBound)
p.L = L; p.W = W;
p.r = 0.5*hypot(L,W);
p.vmax = vmax; p.amax = amax; p.yawmax = yawmax;
p.erratic = erratic; p.vru = vru; p.laneBound = laneBound;
end
