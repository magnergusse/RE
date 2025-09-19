function animate_robot_with_spring(robot, q_trajectory, P1, num_coils, coil_radius)
    % robot: SerialLink robot from Robotics Toolbox
    % q_trajectory: Matrix where each row is a robot joint configuration
    % P1: Fixed attachment point of the spring
    % num_coils: Number of coils in the spring
    % coil_radius: Radius of the coils

    % Get the initial robot end-effector position
    q_init = q_trajectory(1,:) ;  % the first point of the passed trajectory

    T = robot.fkine(q_init); % Forward kinematics
    P2 = T.t'; % Extract translation part

    % Generate the initial spring helix
    [X, Y, Z] = generate_spring(P1, P2, num_coils, coil_radius);
    spring_handle = plot3(X, Y, Z, 'k', 'LineWidth', 2);

    sprK=3;  % spring constant 3 N/m (just an example)

    % Animation loop
    for i = 1:size(q_trajectory, 1)
        q = q_trajectory(i,:);  % point i of the trajectory to execute

        % Update robot configuration
        robot.animate(q);

        % Update end-effector position
        T = robot.fkine(q);
        P2 = T.t'; % New end-effector position to attach the spring

        % Compute new spring shape
        [X, Y, Z] = generate_spring(P1, P2, num_coils, coil_radius);

        % Update the spring plot instead of redrawing
        set(spring_handle, 'XData', X, 'YData', Y, 'ZData', Z);

        sprF=-sprK*(P2-P1)'; % force is proportional to spring elongation
        sprF=[sprF; 0;0;0];% full  generalised force. (0;0;0) are the 3 external moments

        if i==1  %first time
            Hout=DrawTorques(robot,q,sprF); % First call, draw torque vectors
            pause
        else
            DrawTorques(robot,q,sprF,Hout);% subsequent call. Update calls
        end

        pause(0.04); % Smooth animation
    end
end


%---------------------------------------------------------------------
