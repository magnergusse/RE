function drawSatellite(T)
% T is the homogeneous transformation of its location

% Sphere Parameters
r = 0.5; % Radius of the sphere
[Xs, Ys, Zs] = sphere(30); % Generate sphere coordinates

Xs=r*Xs;
Ys=r*Ys;
Zs=r*Zs;

% Panel Parameters
panel_size = [1, 2]; % [Width, Height]
link_length = 1;
panel_offset = r + link_length + panel_size(2) / 2; % Distance from sphere center

% Panel vertices in local coordinates (homogeneous format)
panel_vertices = [...
    -1,  1,  1, -1;
    -1, -1,  1,  1;
     0,  0,  0,  0;
     1,  1,  1,  1]; % Unit square in homogeneous coordinates

% Scale the panel to the correct size
S_panel = makehgtform('scale', [panel_size, 1]); 
panel_vertices = S_panel * panel_vertices; 
% Define colors
link_color = [0.3 0.3 0.3];
panel_color = [0 0 1];
% Transformation Matrices
% Rotate Panels by 90 degrees around the X-axis to align them with the hemisphere plane
R_x90 = makehgtform('xrotate', pi/2);
% Left Panel: Translated along -X axis
T_left = makehgtform('translate', [-panel_offset, 0, 0]) * R_x90;
% Right Panel: Translated along +X axis
T_right = makehgtform('translate', [panel_offset, 0, 0]) * R_x90;
% Left Link: Line from sphere to panel (from -r to -r-link_length along X)
link_left = [-r  -r-link_length; 
              0 0
              0 0
              1 1];
% Right Link: Line from sphere to panel (from r to r+link_length along X)
link_right = [r r+link_length
              0, 0
              0, 0
              1, 1];
% Left Panel
panel_left = T_left * panel_vertices;
% Right Panel
panel_right = T_right * panel_vertices;

% Convert the coordinates with T
% Translations for the sphere
Xs = Xs + T(1,4);
Ys = Ys + T(2,4);
Zs = Zs + T(3,4);
%Full transformation for the others
link_right = T*link_right;
link_left = T*link_left;
panel_right = T*panel_right;
panel_left = T*panel_left;

% Draw Sphere
surf(Xs, Ys, Zs, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none');
%Draw panels
patch(panel_right(1,:), panel_right(2,:), panel_right(3,:), panel_color);
patch(panel_left(1,:), panel_left(2,:), panel_left(3,:), panel_color);
% Draw Links
plot3(link_left(1,:), link_left(2,:), link_left(3,:), 'Color', link_color, 'LineWidth', 3);
plot3(link_right(1,:), link_right(2,:), link_right(3,:), 'Color', link_color, 'LineWidth', 3);

