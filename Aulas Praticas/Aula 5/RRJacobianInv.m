function Ji= RRJacobianInv(robot,Q)
%RRJACOBIANINV Summary of this function goes here
%   Detailed explanation goes here

J=RRjacobian(robot,Q);
a=det(J);%determinante do jacobiano

if abs(a)<eps % um valor muiito pequeno, neste caso o menor que o matlab consegue representar
    Ji=NaN;
else
    Ji=inv(J);


end

