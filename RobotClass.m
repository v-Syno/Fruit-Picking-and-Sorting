%% RobotClass

classdef RobotClass
    %Class for Robot functions

    methods (Static)

        function RobotTeach(robot)
            q = robot.model.getpos(); 
            robot.model.teach(q);
        end

        function Logs(robot, target)
            % Display Transforms
            % With a robot model and target/goal for the EE
            % display the Target EE, currentEE at goal and the estimated
            % joint states required for the target EE pose

            q0 = robot.model.getpos(); % get current joint poses
            T_EE = transl(target) * trotx(pi); % rotate 180 deg so that the Z axis points down
            qGoal = robot.model.ikcon(T_EE,q0); %obtain needed joints for goal
            currentPose = robot.model.fkine(q0).T;

            currentPosition = currentPose(1:3, 4);  % Extract [x, y, z] from the current pose

            fprintf('\nTarget EE pose: [%.3f, %.3f, %.3f] \n', target)
            fprintf('Current EE pose: [%.3f, %.3f, %.3f] \n', currentPosition)
            fprintf('Estiamted Joints for target EE: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] \n', qGoal)
        end

        function qGoal = MoveObject(robot,pose,steps,object,vertices, pickUp, method)
            % Move Robot to given location
            %   Given an object and its goal location,
            %   move the robot using inverse kinematics
            %   update the object's vertices to the new transformation if
            %   robot is holding object only
            %
            % method: 1 = jtraj
            %         2 = trapezoidal

            % gripper = Gripper();
    
            q0 = robot.model.getpos(); % get current joint poses
            T_EE = transl(pose) * trotx(pi); % rotate 180 deg so that the Z axis points down
            qGoal = robot.model.ikcon(T_EE,q0); %obtain needed joints for goal

            if method == 1
                % Jtraj Generation for smoother output
                qMatrix = jtraj(q0,qGoal,steps);

            elseif method == 2
                % Method 2 Trapezoidal Velocity Profile - linear interpolation between points
                s = lspb(0,1,steps);  % First, create the scalar function
                qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
                for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*q0 + s(i)*qGoal;
                end
            end

            % execute motion
            for i = 1:steps
                robot.model.animate(qMatrix(i,:));
                
                % adjustment = transl(0.015,0,0) * trotx(pi/2);
                % T_ee = robot.model.fkineUTS(qMatrix(i,:)) * adjustment;  % Get the current EE transformation
                % gripper.UpdateGripperBase(T_ee);  % Update the gripper's base
                % gripper.PlotGripper();
                
                if pickUp

                    % update objects vertices to reflect its transformation
                    updated_transform = robot.model.fkineUTS(qMatrix(i,:));
                    
                    % apply transpose for matrix multiplication
                    updated_vertices = [vertices, ones(size(vertices, 1), 1)] * updated_transform';
                    set(object, 'Vertices', updated_vertices(:, 1:3));
                end
                drawnow();
            end
            qGoal = qMatrix(end, :);
        end

        function RobotPointCloud(robot,hull)
            % Point cloud generation code (edited point-cloud from lab3 exercise 2)
            % hull: true to see convex hull (volume arm may occupy)
            %       false to hide
            stepRads = deg2rad(45);
            prisStep = 0.7;  % Step size for prismatic joint
            prisStepQty = 3;  % Number of steps for prismatic joint
            qlim = robot.model.qlim;
            pointCloudSize = prod(floor(1 + prisStepQty * ((2 * pi / stepRads)^6 - 1)));  % Assuming 6 revolute joints
            pointCloud = zeros(pointCloudSize, 3);
            counter = 1;
            
            tic
            for q1 = qlim(1,1):(prisStep/prisStepQty):qlim(1,2)  % Prismatic joint, representing the linear rail
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q7 = 0;  % Neutral joint 7
                                    q = [q1, q2, q3, q4, q5, q6, q7];
                                    T = robot.model.fkineUTS(q); % Found in toolbox. Faster than fkine and doesnt require '.T'
                                    pointCloud(counter,:) = T(1:3, 4)';  % Extract translation vector
                                    counter = counter + 1;
                                    if mod(counter / pointCloudSize * 100, 1) == 0
                                        elapsedTime = toc;
                                        progress = (counter/ pointCloudSize) * 100;
                                        fprintf('Seconds: %.2f for Progress: %.2f%%\n', elapsedTime,progress);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            % Calculate the maximum reach and volume (including linear rail)
            distances = sqrt(pointCloud(:,1).^2 + pointCloud(:,2).^2 + pointCloud(:,3).^2);  % Euclidean distance from origin
            maxReach = max(distances);  % Maximum distance
            
            fprintf('Maximum reach (XYZ): %.2f meters\n', maxReach);

            [K,volume] = convhull(pointCloud); % similar to finding the volume of a cylinder.
                                               % convexhull will find the
                                               % volume where all the
                                               % points lie
            fprintf('Approximate volume of workspace: %.2f cubic meters\n', volume);
            
            % Plot the point cloud
            scatter3(pointCloud(:,1), pointCloud(:,2), pointCloud(:,3), 'r.');  % Plot point cloud
            if hull == true
                trisurf(K, pointCloud(:,1), pointCloud(:,2), pointCloud(:,3), 'Facecolor', 'magenta', 'FaceAlpha', 0.3);  % Convex hull
            end

        end

    function qEnd = MoveRobot2(robot, position, steps, payload, holdingObject, vertices, endEffDirection, g_1, g_2, grip, cow)
        % Moves the robot to a specified location and handles object picking/placing if required
        % Arguments:
        %   robot - The robot model
        %   position - The target position for the end effector
        %   steps - Number of steps for the movement
        %   payload - The object to be moved
        %   holdingObject - Boolean, whether the robot is holding an object
        %   vertices - Object vertices for visualization
        %   endEffDirection - The direction the end effector should point
        %   g_1, g_2 - Grippers for the robot (UR3-specific)
        %   grip - Grip state (open/close)
        %   cow - The cow object (for collision testing)
    
        % Initialize the emergency stop and control booleans
        StoreSwitchButtons.setgeteStop(false);
        StoreSwitchButtons.setgetManual(false);
        StoreSwitchButtons.setgetCow(false);
    
        robotcount = 1;
        
        % Define cow collision points
        [Y, X] = meshgrid(-0.2:0.1:0.2, -0.4:0.1:0.4);
        Z = repmat(0.25, size(X));
        cubePoints = [X(:), Y(:), Z(:)];  % Base points for cow's bounding box
        cubePoints = [cubePoints; cubePoints * rotx(pi/2); cubePoints * rotx(pi); cubePoints * rotx(3*pi/2); cubePoints * roty(pi/2); cubePoints * roty(-pi/2)];
        cubePoints = cubePoints + repmat([-1, -0.7, -0.2], size(cubePoints, 1), 1);
    
        % Set the end effector orientation based on direction
        switch endEffDirection
            case 1
                endMove = transl(position) * trotx(-pi/2);  % Towards positive Y axis
            case 2
                endMove = transl(position) * trotx(pi);  % Towards negative Z axis
            case 3
                endMove = transl(position) * trotx(pi/2);  % Towards negative Y axis
            otherwise
                endMove = transl(position) * troty(-pi/2);  % Towards positive X axis
        end
    
        % Calculate joint configurations (q1 and q2)
        q0 = robot.model.getpos();
        q1 = robot.model.ikcon(robot.model.fkine(q0), q0);
        q2 = robot.model.ikcon(endMove, q0);
    
        % Initialize collision functions
        collF = CollisionFunctions();
    
        % Generate trajectory using trapezoidal velocity profile
        s = lspb(0, 1, steps);
        qMatrix = nan(steps, length(robot.model.links));
        for i = 1:steps
            qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2;
        end
    
        % Handle gripper open/close states
        if grip == 1 || grip == 2
            leftQopen = [deg2rad(-20), deg2rad(20), 0];
            rightQopen = [deg2rad(20), deg2rad(-20), 0];
            leftQclosed = [deg2rad(-30), deg2rad(30), 0];
            rightQclosed = [deg2rad(30), deg2rad(-30), 0];
            
            if grip == 1
                qPath1 = jtraj(rightQopen, rightQclosed, steps);
                qPath2 = jtraj(leftQopen, leftQclosed, steps);
            elseif grip == 2
                qPath1 = jtraj(rightQclosed, rightQopen, steps);
                qPath2 = jtraj(leftQclosed, leftQopen, steps);
            end
        end
    
        % Motion execution loop
        for i = 1:steps
            % Emergency stop handling
            [eStopValue, ~] = RobotFunctions.Check_eStop(StoreSwitchButtons.setgeteStop, StoreSwitchButtons.setgetManual, StoreSwitchButtons.setgetCow);
            if eStopValue
                % Save robot and gripper positions
                StopQs = [robot.model.getpos(), g_1.model.getpos(), g_2.model.getpos()];
                RobotFunctions.eStop(StopQs, robotcount, cow);
                disp('Emergency stop triggered!');
                break;
            end
    
            % Cow collision check
            if StoreSwitchButtons.setgetCow()
                cow.model.base = transl(1, -0.5, 0.01);
                cow.model.animate(cow.model.getpos());
                drawnow;
                cowCheck = collF.lightcurtainCheck(cow);
                if cowCheck
                    StoreSwitchButtons.setgeteStop(true);
                end
                StoreSwitchButtons.setgetCow(false);
            end
    
            % Self, ground, and cow collision checks
            selfCheck = collF.collisionCheckSelf(robot, qMatrix(i, :));
            groundCheck = collF.collisionGroundLPI(robot);
            cowCheck = collF.collisionCheckCow(robot, cow);
    
            if selfCheck || groundCheck == 1 || cowCheck
                disp('Collision detected! Adjusting path...');
                % Generate avoidance trajectory and update qMatrix
                newQMatrix = collF.remakeTraj(robot, 10, steps, q2);  % Re-plan trajectory to avoid collision
                qMatrix = newQMatrix;
                i = 1;  % Restart loop to execute new trajectory
                continue;
            end
    
            % Robot and gripper animation
            robot.model.animate(qMatrix(i, :));
            pos1 = robot.model.fkineUTS(robot.model.getpos()) * transl(0, -0.0127, 0.05) * troty(-pi/2);
            pos2 = robot.model.fkineUTS(robot.model.getpos()) * transl(0, 0.0127, 0.05) * troty(-pi/2);
            
            g_1.model.base = pos1;
            g_2.model.base = pos2;
            g_1.model.animate(g_1.model.getpos());
            g_2.model.animate(g_2.model.getpos());
    
            if grip == 1 || grip == 2
                g_1.model.animate(qPath1(i, :));
                g_2.model.animate(qPath2(i, :));
            end
    
            % Update object vertices if holding an object
            if holdingObject
                transMatrix = robot.model.fkine(qMatrix(i, :)).T * transl(0, 0, 0.2);
                transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transMatrix';
                set(payload, 'Vertices', transformedVertices(:, 1:3));
            end
    
            drawnow();
        end
        qEnd = qMatrix(end, :);  % Return the final configuration
        end

    end
end