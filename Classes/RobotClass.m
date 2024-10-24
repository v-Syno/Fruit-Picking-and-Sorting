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

            % Get pose with desired EE
            T_EE = transl(pose) * EEDir;

            % Avoid kinematic singularities
            minBendAngle = 15;
        
            % Get initial and goal joint configurations.
            q0 = robot.model.getpos();
            q0 = AdjustForSingularities(q0, minBendAngle); % Adjust initial configuration.
            qGoal = robot.model.ikcon(T_EE, q0);
            
            % Generate a smoother joint trajectory using `jtraj`.
            qMatrix = jtraj(q0, qGoal, steps);

            % Iterate through the trajectory steps.
            for j = 1:steps

                qCurrent = qMatrix(j, :);   

                % Animate the robot.
                robot.model.animate(qCurrent);

                UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject);

                drawnow();
            end
        end

        function qGoals = DualRobot(robots, poses, steps, payloads, vertices, holdingObject, grippers, EEDir)
            % robots: Cell array of robots (e.g., {robot1, robot2})
            % poses: Cell array of desired end-effector positions (e.g., {[x1, y1, z1], [x2, y2, z2]})
            % steps: Number of steps for the trajectory.
            % payloads: Cell array of payloads (e.g., {payload1, payload2}).
            % vertices: Cell array of object vertices (e.g., {vertices1, vertices2}).
            % holdingObject: Array of logicals indicating if each robot is holding an object.
            % grippers: Cell array of grippers (e.g., {{g1, g2}, {g3, g4}}).
            % EEDir: Cell array of end-effector orientations (e.g., {EEDir1, EEDir2}).
        
            % Initialize object class for updating positions.
            object = ObjectClass();
        
            % Number of robots.
            numRobots = length(robots);
            qGoals = cell(1, numRobots);
            qMatrices = cell(1, numRobots);
        
            % Compute goal configurations and trajectories for each robot.
            for r = 1:numRobots
                robot = robots{r};
                T_EE = transl(poses{r}) * EEDir{r};
                q0 = robot.model.getpos();
                qGoals{r} = robot.model.ikcon(T_EE, q0);
                qMatrices{r} = jtraj(q0, qGoals{r}, steps);
            end
        
            % Iterate through the trajectory steps.
            for i = 1:steps
                % Loop through each robot to animate and update positions.
                for r = 1:numRobots
                    robot = robots{r};
                    qMatrix = qMatrices{r};
                    payload = payloads{r};
                    vert = vertices{r};
                    isHolding = holdingObject(r);
                    gripper = grippers{r};
        
                    % Animate the robot.
                    robot.model.animate(qMatrix(i, :));
        
                    % Update gripper positions for the robot.
                    pos1 = robot.model.fkineUTS(qMatrix(i, :)) * transl(0, -0.0127, 0.05) * troty(-pi/2);
                    pos2 = robot.model.fkineUTS(qMatrix(i, :)) * transl(0, 0.0127, 0.05) * troty(-pi/2);
                    gripper{1}.model.base = pos1;
                    gripper{2}.model.base = pos2;
                    gripper{1}.model.animate(gripper{1}.model.getpos());
                    gripper{2}.model.animate(gripper{2}.model.getpos());
        
                    % Update object positions if the robot is holding an object.
                    if isHolding
                        object.UpdateObjectPosition(robot, qMatrix(i, :), payload, vert);
                    end
                end
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

    end
end