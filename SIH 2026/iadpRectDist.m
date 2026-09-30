function [dist, overlap] = iadpRectDist(pose1, size1, pose2, size2)
%IADPRECTDIST  Distance between two oriented rectangles (0 if they overlap).
%
%   [dist, overlap] = iadpRectDist([x y psi], [L W], [x y psi], [L W])
%
%   Used for collision detection and clearance metrics. Discs are fine for
%   planning speed, but reporting "minimum clearance" with a disc model
%   around an 11 m bus would be meaningless, so the metrics use this.

V1 = rectVerts(pose1, size1);
V2 = rectVerts(pose2, size2);

overlap = satOverlap(V1, V2);
if overlap
    dist = 0; return;
end

dist = min(polyDist(V1, V2), polyDist(V2, V1));
end

% =======================================================================
function V = rectVerts(p, sz)
c = cos(p(3)); s = sin(p(3));
L = sz(1)/2;  W = sz(2)/2;
loc = [ L  W; L -W; -L -W; -L  W];
V = [p(1) + loc(:,1)*c - loc(:,2)*s, p(2) + loc(:,1)*s + loc(:,2)*c];
end

function tf = satOverlap(A, B)
tf = true;
for which = 1:2
    if which == 1, P = A; else, P = B; end
    for k = 1:2                    % rectangles: 2 unique axes each
        e = P(mod(k,4)+1,:) - P(k,:);
        ax = [-e(2) e(1)];  ax = ax/max(norm(ax),1e-9);
        a = A*ax.';  b = B*ax.';
        if max(a) < min(b) - 1e-9 || max(b) < min(a) - 1e-9
            tf = false; return;
        end
    end
end
end

function d = polyDist(A, B)
d = inf;
for i = 1:4
    p = A(i,:);
    for j = 1:4
        q1 = B(j,:); q2 = B(mod(j,4)+1,:);
        d = min(d, ptSegDist(p, q1, q2));
    end
end
end

function d = ptSegDist(p, a, b)
ab = b - a; ap = p - a;
L2 = ab*ab.';
if L2 < 1e-12
    d = norm(ap); return;
end
t = max(0, min(1, (ap*ab.')/L2));
d = norm(ap - t*ab);
end
