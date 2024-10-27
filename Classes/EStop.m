classdef EStop

    methods(Static)
        function status = GlobalEStop(flag)
            % GlobalEStop manages the global E-Stop state
            % If a flag is provided, it toggles the E-Stop state.
            % Otherwise, it returns the current state.
            
            persistent eStopActive % Persistent variable to hold E-Stop state
            if isempty(eStopActive)
                eStopActive = false; % Initialize as false
            end
            
            if nargin > 0
                % Toggle state if flag is provided
                eStopActive = flag;
            end
            
            status = eStopActive;
        end

        function CheckJoystickEStop(buttonID)
            % CheckJoystickEStop Toggles the E-Stop state when a specified joystick button is pressed.
            % Inputs:
            %   - joyID: Joystick ID number (e.g., 1 for the first joystick).
            %   - buttonID: The button number to use for the E-Stop toggle (e.g., 2).
            
            % Define the joystick based on the provided ID
            joy = vrjoystick(1);
            
            % Define a persistent variable to track the button's pressed state
            persistent buttonPressed;
            
            if isempty(buttonPressed)
                buttonPressed = false; % Initialize button state if undefined
            end
            
            % Read the current button states
            [~, buttons, ~] = read(joy);
            
            % Check if the specified button is pressed and wasn't pressed before
            if buttons(buttonID) && ~buttonPressed
                % Toggle the E-Stop state
                RobotClass.ToggleEStop();
                
                % Set buttonPressed to true to avoid repeated toggling
                buttonPressed = true;
                
                % Display the E-Stop status
                if EStop.GlobalEStop()
                    disp('E-Stop Activated by joystick');
                else
                    disp('E-Stop Deactivated by joystick');
                end
                
            % Reset the buttonPressed flag once the button is released
            elseif ~buttons(buttonID) && buttonPressed
                buttonPressed = false;
            end
        end

    end
end