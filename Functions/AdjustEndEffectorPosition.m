function qAdjusted = AdjustEndEffectorPosition(robot, qCurrent, targetPose, EEDir, tolerance, maxIterations)
    % Create the target transformation matrix for the desired pose.
    T_EE_desired = transl(targetPose) * EEDir;
    
    % Initialize the current joint configuration.
    qAdjusted = qCurrent;
    
    % Calculate the initial transformation of the end-effector.
    T_EE_current = robot.model.fkineUTS(qAdjusted);
    
    % Calculate the position and orientation error.
    deltaPose = tr2delta(T_EE_current, T_EE_desired);
    positionError = norm(deltaPose(1:3)); % Position error (x, y, z)

    iteration = 0;
    while positionError > tolerance && iteration < maxIterations
        % Use `ikcon` to find a new configuration that reduces the error.
        % Start from the current configuration for a local solution.
        qAdjusted = robot.model.ikcon(T_EE_desired, qAdjusted);
        
        % Recalculate the end-effector transformation and error.
        T_EE_current = robot.model.fkineUTS(qAdjusted);
        deltaPose = tr2delta(T_EE_current, T_EE_desired);
        positionError = norm(deltaPose(1:3));
        
        % Update iteration count.
        iteration = iteration + 1;
    end
end
