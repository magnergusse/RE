function h=drawFTvector(P,v,scale,color,label,prevH)
% Auxiliary function to draw a vector with some parameters
% h - the graphic handle for potencial future updates
% P - the 3D point where to draw the vector origin
% v - the 3D vector to draw
% scale - (optional) a scale factor to reduce the actual length of the vector
% color - (optional) The color of the vector (like 'r', 'b', 'g', 'm', etc.)
% label - (optional) A label to add to near the tip of the vector
% prevH - (optional) the graphic handle to use (if present do not draw a
%         new vector but update the coordinates of a previsouly drawn vector.)
% vsantos-UA, 2025

newPlot=0;
if nargin < 6, newPlot=1;end
if nargin < 5, label=''; end
if nargin < 4, color = 'r';end
if nargin < 3, scale=0.5;end

x=P(1); y=P(2); z=P(3);
vx=v(1);vy=v(2);vz=v(3);
if newPlot
    h=quiver3(x, y,z, vx, vy, vz, color,...
              'AutoScaleFactor',scale,...
              'MaxHeadSize',3,...
              'LineWidth',3);
%You can adjust parameters like the arrow tip size, the line width, etc.
    LLoc=P+v*scale + 0.18*v/norm(v);
    text(LLoc(1), LLoc(2), LLoc(3),label,'Color',color);
    %Optionally add some text to identify the vector
else
    prevH.XData=x;  prevH.YData=y;  prevH.ZData=z;
    prevH.UData=vx; prevH.VData=vy; prevH.WData=vz;
    h=prevH;
end
