function P = potential_energy(robot, q)
% total potential energy of robot for joints
% configuration q
n = robot.n; % Number of joints
P = 0; % Initialize potential energy
g = robot.gravity'; % gravity vector
r = computeCoM(robot,q); % absolute CoM
% Loop over all links
for i = 1:n
% Get the mass and CoM of the current link
m_i = robot.links(i).m;
r_i = r(:,i);
% Add the contribution to potential energy
P = P + (-g*r_i*m_i); % the formula on the right -->
end