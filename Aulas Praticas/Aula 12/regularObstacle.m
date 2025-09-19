function shape = regularObstacle(n, radius, center, orientation)
% Returns the vertices of a regular polygon
% n          : number of sides (n ≥ 3)
% radius     : radius of the circumscribing circle
% orientation: rotation angle in radians (0 aligns one vertex with +x axis)
if nargin < 4
    orientation = 0;
end
if nargin < 3
    center = [0 0];
end
if nargin < 2
    radius = 1;
end
if nargin < 1
    n=3;
end

if n < 3
    n=3;
end

theta = (0:n-1) * (2*pi/n) + orientation;
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
shape =  polyshape(x, y);
end
