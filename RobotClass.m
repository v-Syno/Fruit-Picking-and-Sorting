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

        function qGoal = MoveObject(robot,pose,steps,object,vertices,pickUp, method)
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

        function qGoal = MoveObject2(robot, pose, steps, object, vertices, pickUp)
            % MoveObject2 Moves a robot to a specified pose using a chosen method
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

        function qEnd = MoveRobot(robot, position, steps, payload, holdingObject, vertices, varargin)
            % MoveRobot moves the robot to a specified position with specified options.
            % Arguments:
            %   robot - The robot model.
            %   position - The target position for the end effector [x, y, z].
            %   steps - Number of steps for the movement.
            %   payload - The object to be moved.
            %   holdingObject - Boolean indicating if the robot is holding an object.
            %   vertices - Vertices of the payload for visualization.
            %   'endEffDirection' - Optional argument specifying the end-effector direction.
            %                       Accepts 'up', 'down', 'left', 'right', 'forward', 'backward'.
            %                       Default is 'down'.
        
            % Default values for options
            endEffDirection = 'down';  % Default end-effector direction
        
            % Parse optional arguments
            if nargin > 6
                for i = 1:2:length(varargin)
                    switch lower(varargin{i})
                        case 'endeffdirection'
                            endEffDirection = validatestring(varargin{i + 1}, ...
                                {'up', 'down', 'left', 'right', 'forward', 'backward'});
                        otherwise
                            error('Unknown parameter: %s', varargin{i});
                    end
                end
            end
        
            % Determine the transformation based on end-effector direction
            switch endEffDirection
                case 'up'
                    endMove = transl(position) * trotx(-pi/2);
                case 'down'
                    endMove = transl(position) * trotx(pi);
                case 'left'
                    endMove = transl(position) * troty(pi/2);
                case 'right'
                    endMove = transl(position) * troty(-pi/2);
                case 'forward'
                    endMove = transl(position) * trotx(0); % Point along x-axis
                case 'backward'
                    endMove = transl(position) * trotx(pi);
            end
        
            % Initial robot position and configuration
            q0 = robot.model.getpos();
            q1 = robot.model.ikcon(robot.model.fkine(q0), q0);
            q2 = robot.model.ikcon(endMove, q0);
        
            % Trajectory generation using trapezoidal velocity profile
            s = lspb(0, 1, steps);  % Scalar function for smooth trajectory
            qMatrix = nan(steps, length(robot.model.links));  % Preallocate memory for the trajectory
            for i = 1:steps
                qMatrix(i, :) = (1 - s(i)) * q1 + s(i) * q2;
            end
        
            % Initialize collision detection
            collF = CollisionClass();
        
            % Motion execution loop
            for i = 1:steps
                % Check for collisions with self, ground, or other objects (e.g., cow)
                selfCheck = collF.collisionCheckSelf(robot, qMatrix(i, :));
                groundCheck = collF.collisionGroundLPI(robot);
                
                % If any collision is detected, recompute the trajectory to avoid
                if selfCheck || groundCheck == 1
                    disp('Collision detected! Adjusting path...');
                    newQMatrix = collF.remakeTraj(robot, 10, steps, q2);  % Re-plan trajectory to avoid collision
                    qMatrix = newQMatrix;
                    i = 1;  % Restart loop with new trajectory
                    continue;
                end
        
                % Animate the robot along the trajectory
                robot.model.animate(qMatrix(i, :));
        
                % Update object vertices if holding an object
                if holdingObject
                    transMatrix = robot.model.fkine(qMatrix(i, :)).T * transl(0, 0, 0.2);
                    transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transMatrix';
                    set(payload, 'Vertices', transformedVertices(:, 1:3));
                end
        
                drawnow();  % Update the animation
            end
        
            % Return the final joint configuration
            qEnd = qMatrix(end, :);
        end

        function GripperMove(g1, g2, varargin)
            % GripperMove controls the opening and closing of a gripper using two gripper models (g1 and g2).
            % Arguments:
            %   g1 - The first gripper model.
            %   g2 - The second gripper model.
            %   'goal' - Specify 'open' or 'close' as a string, or true/false as a logical.
            %            - 'open' or true will open the gripper.
            %            - 'close' or false will close the gripper.
            %            - Default is 'close' if not specified.
        
            gsteps = 20;  % Number of steps for the movement.
        
            % Define initial open and closed states for the grippers.
            Initial_leftQopen = zeros(1, 3);
            Initial_rightQopen = zeros(1, 3);
            Initial_leftQclosed = [deg2rad(-20), deg2rad(20), 0];
            Initial_rightQclosed = [deg2rad(20), deg2rad(-20), 0];
        
            % Set default goal to 'close'
            goal = 'open';
        
            % Parse input arguments for the goal
            if nargin > 2
                inputGoal = varargin{1};
                if ischar(inputGoal)
                    % If the goal is provided as 'open' or 'close'
                    goal = validatestring(inputGoal, {'open', 'close'});
                elseif islogical(inputGoal)
                    % If the goal is provided as true or false
                    if inputGoal
                        goal = 'open';
                    else
                        goal = 'close';
                    end
                else
                    error('Invalid goal input. Use ''open'', ''close'', true, or false.');
                end
            end
        
            % Generate the joint trajectories for the grippers based on the goal.
            if strcmp(goal, 'close')
                % Close Gripper
                qPath1 = jtraj(Initial_rightQopen, Initial_rightQclosed, gsteps);
                qPath2 = jtraj(Initial_leftQopen, Initial_leftQclosed, gsteps);
            elseif strcmp(goal, 'open')
                % Open Gripper
                qPath1 = jtraj(Initial_rightQclosed, Initial_rightQopen, gsteps);
                qPath2 = jtraj(Initial_leftQclosed, Initial_leftQopen, gsteps);
            end
        
            % Animate the gripper movements.
            for i = 1:gsteps
                g1.model.animate(qPath1(i, :));
                g2.model.animate(qPath2(i, :));                
                drawnow();
                pause(0.001);  % Small pause for smooth animation.
            end
        end
    end
end