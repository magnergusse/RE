function Q=invKinRRplanar(robot, target)

Q=NaN;

if ~isa(robot, 'SerialLink')
    return
end

if ~robot.config=='RR'
    return
end


x=target(1);
y=target(2);
z=target(3);

L1=robot.a(1);
L2=robot.a(2);



q2=acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));%tetha 2

if imag(q2) ~=0
      Q=[NaN NaN; NaN NaN];
      return 
end


q2=[q2, -q2];%Angulo positivo e negativo para redundancia



q1= atan2(y*(L1+L2*cos(q2))-x*L2*sin(q2), x*(L1+L2*cos(q2))+y*L2*sin(q2));


 Q=[q1;q2];

end