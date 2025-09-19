function CoM_abs = computeCoM(robot, q)
    % Computes absolute CoM positions for a given robot and joint configuration q.
    % Inputs:
    %   robot - SerialLink object (Peter Corke’s toolbox)
    %   q - Joint configuration (1xn)
    % Output:
    %   CoM_abs - Absolute CoM positions in base frame (3xn matrix)

    CoM_abs = zeros(3, robot.n); % Store results
    for i = 1:robot.n
        T_ii = robot.A(1:i-1,q);     % The full previous transformation    
        R_theta = SE3(trotz(q(i)));  % Obtain only rotation about z for current link

        % Compute CoM in absolute frame
        CoM_i = T_ii * R_theta * robot.links(i).r(:); % ensure column vector

        % If prismatic, add translation along z-axis
        if robot.links(i).isprismatic % Check if joint is prismatic
            CoM_i = CoM_i + q(i) * T_ii.a;
        end
        % Store result
        CoM_abs(:,i) = CoM_i;
    end
end
%% Aux function to ensure that this is the one run and not anything else on the path!
function M=trotz(a) 
M=[cos(a)  -sin(a) 0 0
   sin(a)   cos(a) 0 0
   0          0    1 0
   0          0    0 1];
end