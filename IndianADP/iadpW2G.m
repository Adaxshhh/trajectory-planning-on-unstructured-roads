function [c, r, ok] = iadpW2G(map, x, y)
%IADPW2G  World (x,y) -> grid column/row. ok=false when outside the map.
c = floor((x - map.xmin)/map.res) + 1;
r = floor((y - map.ymin)/map.res) + 1;
ok = c >= 1 & c <= map.nx & r >= 1 & r <= map.ny;
c = min(max(c,1), map.nx);
r = min(max(r,1), map.ny);
end
