classdef RobotClass
    % Class for Robot functions

    methods (Static)

        function qGoal = MoveRobot(robot, pose, steps, payload, vertices, holdingObject,g1,g2,EEDir)

            joystickConnected = true;
            try
                joy = vrjoystick(1);
            catch
                joystickConnected = false;  % Set flag if joystick is not available
            end

            % Get pose with desired EE
            T_EE = transl(pose) * EEDir;

            % Avoid kinematic singularities
            minBendAngle = 15;
        
            % Get initial and goal joint configurations.
            q0 = robot.model.getpos();
            q0 = RobotClass.AdjustForSingularities(q0, minBendAngle);
            qGoal = robot.model.ikcon(T_EE, q0);
            
            % Generate a smoother joint trajectory using `jtraj`.
            qMatrix = jtraj(q0, qGoal, steps);

            % Iterate through the trajectory steps.
            % for i = 1:steps
            i = 1;
            while i <= steps
                % Check E-Stop state
                % if RobotClass.CheckEStop()
                %     disp('Paused due to E-Stop');
                %     while RobotClass.CheckEStop()
                %         pause(0.1); % Maintain pause until E-Stop is deactivated
                %     end
                % end

                % Call the joystick E-Stop check function
                if joystickConnected
                    CheckJoystickEStop();
                end

                if RobotClass.CheckEStop()
                    pause(0.1); % Maintain pause until E-Stop is toggled off
                    continue; % Skip movement if E-Stop is active
                end

                qCurrent = qMatrix(i, :);
                qCurrentAdjusted = RobotClass.AdjustForSingularities(qCurrent, minBendAngle);

                % Animate the robot.
                robot.model.animate(qCurrentAdjusted);

                RobotClass.UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject);

                i = i + 1;
                drawnow();

            end
        end

        function GripperMove(right, left, state)
            % Controls the opening or closing of the gripper.
            steps = 50; % Number of steps for smoother transition.
            
            % Define angles for open/closed states.
            leftQopen = [deg2rad(-20), deg2rad(20), 0];
            rightQopen = [deg2rad(20), deg2rad(-20), 0];
            leftQclosed = [deg2rad(-30), deg2rad(30), 0];
            rightQclosed = [deg2rad(30), deg2rad(-30), 0];
        
            if strcmp(state, 'close')
                qPath1 = jtraj(rightQopen, rightQclosed, steps);
                qPath2 = jtraj(leftQopen, leftQclosed, steps);
            elseif strcmp(state, 'open')
                qPath1 = jtraj(rightQclosed, rightQopen, steps);
                qPath2 = jtraj(leftQclosed, leftQopen, steps);
            end
        
            % Animate the gripper.
            for i = 1:steps
                right.model.animate(qPath1(i, :));
                left.model.animate(qPath2(i, :));
                drawnow();
            end
        end

        function qAdjusted = AdjustForSingularities(qCurrent, minBendAngle)
            qAdjusted = qCurrent;
            
            % Adjust the 3rd joint to ensure the elbow maintains a minimum bend.
            % This is common for robots like UR3e where the third joint represents the elbow.
            if abs(qCurrent(3)) < deg2rad(minBendAngle)
                qAdjusted(3) = sign(qCurrent(3)) * deg2rad(minBendAngle);
            end
        end

        function UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject)
        
            object = ObjectClass();
        
            % Gripper base transform for UR3.
            pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0.0127,0,0.0612)*trotx(pi/2)*trotz(pi/2);%z0.0612
            pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(-0.0127,0,0.0612)*trotx(pi/2)*trotz(pi/2);%z0.0612
            
            g1.model.base = pos1; 
            g2.model.base = pos2; 
            g1.model.animate(g1.model.getpos());
            g2.model.animate(g2.model.getpos());
            
            % Update object position if holding.
            if holdingObject
                object.UpdateObjectPosition(robot, qCurrent, payload, vertices);
            end
        
        end

        function ToggleEStop()
            % Toggle the E-Stop state using the global function
            currentState = GlobalEStop();
            GlobalEStop(~currentState); % Toggle E-Stop state
            if GlobalEStop()
                disp('E-Stop Activated');
            else
                disp('E-Stop Deactivated, resuming movement...');
            end
        end
        
        function state = CheckEStop()
            % Return the current E-Stop state from GlobalEStop
            state = GlobalEStop();
        end

    end
end