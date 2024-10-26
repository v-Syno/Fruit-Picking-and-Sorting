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
            q0 = RobotClass.AdjustForSingularities(q0, minBendAngle); % Adjust initial configuration.
            qGoal = robot.model.ikcon(T_EE, q0);
            
            % Generate a smoother joint trajectory using `jtraj`.
            qMatrix = jtraj(q0, qGoal, steps);

            % Iterate through the trajectory steps.
            for j = 1:steps

                qCurrent = qMatrix(j, :);   

                % Animate the robot.
                robot.model.animate(qCurrent);

                RobotClass.UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject);

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

    end
end