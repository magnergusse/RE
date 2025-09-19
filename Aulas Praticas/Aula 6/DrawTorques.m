function Hout=DrawTorques(robot,q,Fext,Hin)

% Function to draw and update vector representing torques in joints
% It accepts the robot, its position, the external Force applied
% and an optional array of graphic handles, This is useful when updating
% graphics and not creating new ones.
% It returns graphic handles mainly for the first call and useful for
% subsequent calls.
% The function can be enhanced for other features...
% vsantos-UA, 2025

if nargin < 4
    newplot=1;
else
    newplot=0;
end

Fscale=0.1;  % This is hardcoded. You can adapt or pass as argument.

J=robot.jacob0(q);
tau = J' * Fext;

axe1=[0 0 1]';
P1=[0 0 0]';
if newplot
    Hout(1)=drawFTvector(P1,tau(1)*axe1,Fscale,'m', '\tau_1');
else
    drawFTvector(P1,tau(1)*axe1,Fscale,'m', '',Hin(1));
end

for i=1:robot.n-1
    T=robot.A(1:i,q);
    P=T.t;
    axe=T.a;
    if newplot
        label="\tau_"+(i+1);
        Hout(i+1)=drawFTvector(P,tau(i)*axe,Fscale,'m', label);
    else
        drawFTvector(P,tau(i)*axe,Fscale,'m', '',Hin(i+1));
    end
end