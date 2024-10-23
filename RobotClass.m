%% RobotClass
classdef RobotClass
    % Class for Robot functions

    methods (Static)

        function qGoal = MoveObject(robot, pose, steps, object, vertices, pickUp)
            % MoveObject2 Moves a robot to a specified pose
            % 
            % Inputs:
            %   - robot: The robot model.
            %   - pose: Desired end-effector pose
            %   - steps: Number of steps for the trajectory.
            %   - object: The graphical object to update.
            %   - vertices: Vertices of the object.
            %   - pickUp: Boolean indicating if the robot is holding the object.
            %
            % Outputs:
            %   - qGoal: Final joint configuration.
            
        
            q0 = robot.model.getpos(); % Get current joint poses
            T_EE = transl(pose)* trotx(pi); % Using the provided end-effector pose
            qGoal = robot.model.ikcon(T_EE, q0); % Compute joint configuration for the goal
        
            qMatrix = jtraj(q0,qGoal,steps);

            % Execute the motion
            for i = 1:steps
                robot.model.animate(qMatrix(i, :));
        
                if pickUp
                    % Update the object's vertices to reflect its transformation
                    updated_transform = robot.model.fkineUTS(qMatrix(i, :));
                    updated_vertices = [vertices, ones(size(vertices, 1), 1)] * updated_transform';
                    set(object, 'Vertices', updated_vertices(:, 1:3));
                end
                drawnow();
            end
        
            qGoal = qMatrix(end, :);
        end

        function qGoal = MoveRobot(robot, pose, steps, payload, vertices, holdingObject,g1,g2,EEDir)

            % Initialise classes
            object = ObjectClass();
            collision = CollisionClass();
            
            % Get pose with desired EE
            T_EE = transl(pose) * EEDir;

            % Avoid kinematic singularities
            minBendAngle = 5;
        
            % Get initial and goal joint configurations.
            q0 = robot.model.getpos();
            q0 = AdjustForSingularities(q0, minBendAngle); % Adjust initial configuration.
            qGoal = robot.model.ikcon(T_EE, q0);
            
            % Generate a smoother joint trajectory using `jtraj`.
            qMatrix = jtraj(q0, qGoal, steps);

            % Define tolerance and adjustment parameters.
            tolerance = 0.01; % Maximum allowable position error.
            maxIterations = 5; % Max iterations for corrections.

            % Iterate through the trajectory steps.
            for i = 1:steps

                qCurrent = qMatrix(i, :);   

                % Check for self-collision.
                if isSelfCollision(robot, qCurrent, g1, g2)
                    % If a collision is detected, adjust the orientation.
                    qCurrent = AdjustPathForCollision(robot, qCurrent, pose, EEDir);
                end

                % Animate the robot.
                robot.model.animate(qCurrent);

                UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject);

                drawnow();
            end
            qGoal = AdjustEndEffectorPosition(robot, qGoal, pose, EEDir, tolerance, maxIterations);
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
    end
end