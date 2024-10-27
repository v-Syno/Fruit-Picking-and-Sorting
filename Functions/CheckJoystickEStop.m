function CheckJoystickEStop()
    persistent joy buttonPressed
    
    % Initialize joystick if it’s the first call
    if isempty(joy)
        joy = vrjoystick(1); % Initialize joystick (e.g., index 1)
        buttonPressed = false; % Flag to track button state
    end

    % Read joystick state
    [~, buttons, ~] = read(joy); % Read button states

    % Check if button 2 for E-Stop has been pressed
    if buttons(2) && ~buttonPressed
        RobotClass.ToggleEStop(); % Toggle E-Stop on button press
        buttonPressed = true; % Mark button as pressed

    elseif ~buttons(2) && buttonPressed
        % Reset flag when button is released
        buttonPressed = false;
    end
end
