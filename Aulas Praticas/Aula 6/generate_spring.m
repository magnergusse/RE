function [X, Y, Z] = generate_spring(P1, P2, num_coils, coil_radius)
% Function to generate the points of spring between two points P1 and P2
% with a given number of coils and coil radius
% vsantos-UA, 2025

% Compute direction and length
L = norm(P2 - P1);
direction = (P2 - P1) / L;

% Create helix along Z-axis in local frame
theta = linspace(0, 2 * pi * num_coils, num_coils * 40);
z = linspace(0, L, numel(theta));
x = coil_radius * cos(theta);
y = coil_radius * sin(theta);
spring_points = [x; y; z];

% Compute rotation matrix
z_axis = direction;
[~, min_idx] = min(abs(z_axis));
v = zeros(3,1); v(min_idx) = 1;
x_axis = cross(v, z_axis); x_axis = x_axis / norm(x_axis);
y_axis = cross(z_axis, x_axis);
R = [x_axis' y_axis' z_axis'];

% Apply transformation
rotated_spring = R * spring_points + P1(:);

% Extract X, Y, Z coordinates
X = rotated_spring(1, :);
Y = rotated_spring(2, :);
Z = rotated_spring(3, :);
end