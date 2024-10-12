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

    end
end