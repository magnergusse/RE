function J=RRjacobian(robot,Q)

LA=robot.a(1);
LB=robot.a(2);

q1=Q(1);
q2=Q(2);


J = [-LB*sin(q1 + q2) - LA*sin(q1), -LB*sin(q1 + q2)
      LB*cos(q1 + q2) + LA*cos(q1),  LB*cos(q1 + q2)];
end