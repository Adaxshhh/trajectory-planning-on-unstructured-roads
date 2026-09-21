function [x, y] = iadpG2W(map, c, r)
%IADPG2W  Grid column/row -> world (x,y) at the cell centre.
x = map.xmin + (c - 0.5)*map.res;
y = map.ymin + (r - 0.5)*map.res;
end
