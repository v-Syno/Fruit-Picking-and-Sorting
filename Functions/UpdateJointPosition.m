function UpdateJointPosition(app, jointIndex, sliderValue)
    % Converts slider value to radians and updates the selected robot's joint
    % configuration for the specified joint index.
    
    if isempty(app.selectedRobot)
        disp('No robot selected for manual control.');
        return;
    end

    % Convert the slider value (assumed to be in degrees) to radians
    jointAngle = deg2rad(sliderValue);
    
    % Get the current joint configuration and update the specified joint
    qCurrent = app.selectedRobot.model.getpos();
    qCurrent(jointIndex) = jointAngle;
    
    % Update the robot's joint configuration
    app.selectedRobot.model.animate(qCurrent);
end
