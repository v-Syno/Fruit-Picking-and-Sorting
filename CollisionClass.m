classdef CollisionClass
    properties (Access = protected)
        ellipX; 
        ellipY;
        ellipZ;
        jointX;
        jointY;
        linkLeng;
        JointEllipse;
        centreJoints;
    end
    
    methods
        function obj = CollisionClass()
            % Constructor for CollisionClass.
            % It can initialize any required parameters or properties.
        end

        function visualiseEllips(obj, robot, dur)
            % Visualizes ellipsoids around the robot links for a given duration.
            hold on
            q0 = robot.model.getpos;
            jointTr = robot.model.fkine(q0);

            % Update joint-specific properties based on the robot type.
            obj.updateJointProperties(robot, jointTr);

            % Generate and display ellipsoids for each link.
            for j = 1:length(obj.centreJoints)
                [obj.ellipX, obj.ellipY, obj.ellipZ] = ellipsoid(obj.centreJoints{j}(1), obj.centreJoints{j}(2), obj.centreJoints{j}(3), obj.jointX, obj.jointY, obj.linkLeng(j));
                obj.JointEllipse(j) = surf(obj.ellipX, obj.ellipY, obj.ellipZ); % Create the ellipsoid.
                Jrot = rad2deg(jointTr(j).tr2rpy);
                % Apply rotations to align the ellipsoids with the link orientation.
                rotate(obj.JointEllipse(j), [0 0 1], Jrot(3), obj.centreJoints{j});
                rotate(obj.JointEllipse(j), [0 1 0], Jrot(2), obj.centreJoints{j});
                rotate(obj.JointEllipse(j), [1 0 0], Jrot(1), obj.centreJoints{j});
            end

            pause(dur);
            delete(obj.JointEllipse);
        end
        
        function updateEllips(obj, robot)
            % Updates the position of ellipsoids based on the robot's current pose.
            q0 = robot.model.getpos;
            jointTr = robot.model.fkine(q0);

            % Update joint-specific properties based on the robot type.
            obj.updateJointProperties(robot, jointTr);
        end

        function updateJointProperties(obj, robot, jointTr)
            % Helper method to update joint properties based on the robot type.
            obj.centreJoints = cell(1, length(jointTr) - 1);
            for i = 2:length(jointTr)
                centre = 0.5 * (jointTr(i - 1).t + jointTr(i).t);
                obj.centreJoints{i - 1} = centre;
            end

            % Adjust joint-specific properties based on the robot type.
            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                obj.jointX = 0.058;
                obj.jointY = 0.058;
                obj.linkLeng = [0.15, 0.001, 0.110, 0.071, 0.095, 0.131, 0.095, 0.091, 0.051, 0.041, 0.001];
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3e')
                obj.jointX = 0.058;
                obj.jointY = 0.058;
                obj.linkLeng = [0.12, 0.158, 0.0935, 0.031, 0.032, 0.0853, 0.031];
            else
                error('Unsupported robot type.');
            end
        end

        function isCollision = collisionCheckSelf(obj, robot, Q)
            % Checks for collisions between the end effector at Q and robot links.
            isCollision = false;
            obj.updateEllips(robot);
            point = robot.model.fkineUTS(Q) * transl(0, 0, 0.15);
            gripPoints = obj.generateGripPoints(point);

            % Check each grip point against each ellipsoid.
            for i = 1:length(obj.centreJoints)
                for j = 1:size(gripPoints, 1)
                    point = gripPoints(j, :)';
                    result = (point - obj.centreJoints{i})' * diag([1/obj.jointX^2, 1/obj.jointY^2, 1/obj.linkLeng(i)^2]) * (point - obj.centreJoints{i});
                    if result <= 1
                        isCollision = true;
                        return;
                    end
                end
            end
        end

        function gripPoints = generateGripPoints(obj, point)
            % Generates points along the gripper for collision checking.
            point1 = point * transl(0, 0.0127, 0.2312);
            point2 = point * transl(0, -0.0127, 0.2312);
            pont = point * transl(0, 0, 0.05);
            point1 = point1(1:3, 4);
            point2 = point2(1:3, 4);
            pont = pont(1:3, 4);
            grip1 = [linspace(pont(1), point1(1), 4)', linspace(pont(2), point1(2), 4)', linspace(pont(3), point1(3), 4)'];
            grip2 = [linspace(pont(1), point2(1), 4)', linspace(pont(2), point2(2), 4)', linspace(pont(3), point2(3), 4)'];
            gripPoints = [grip1; grip2];
        end

        function check = collisionGroundLPI(obj, robot)
            % collisionGroundLPI checks if the robot's gripper intersects with the ground plane.
            planeNormal = [0, 0, 0.1];
            pointOnPlane = [0, 0, 0];  % Ground plane is at z = 0.

            T_EE = robot.model.fkineUTS(robot.model.getpos());
        
            % Define the line segment points based on the robot type.
            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                leftGripper = T_EE * transl(0,-0.127,0.215);
                rightGripper = T-EE * transl(0,0.127,0.215);

            elseif strcmp(robot.plyFileNameStem, 'LinearUR3e')
                leftGripper = T_EE * transl(0,0.127,0.2312);
                rightGripper = T_EE * transl(0,-0.127,0.2312);
            else
                error('Unsupported robot type.');
            end

            point1 = leftGripper(1:3,4)' ;
            point2 = rightGripper(1:3,4)' ;
        
            % Use the LinePlaneIntersection function to find the intersection.
            [~, check] = LinePlaneIntersection(planeNormal, pointOnPlane, point1, point2);
       
        end

        function qPath = RRT(obj,robot, qStart, qGoal, maxIterations, stepSize)
            % RRT Algorithm for path planning
            % Inputs:
            %   - robot: The robot model.
            %   - qStart: Starting joint configuration.
            %   - qGoal: Goal joint configuration.
            %   - maxIterations: Maximum number of iterations for building the tree.
            %   - stepSize: Step size for moving towards random samples.
            % Output:
            %   - qPath: The collision-free path from start to goal.
        
            % Initialize the RRT tree and parent relationship
            tree = qStart;
            parent = [0]; % 0 indicates the root has no parent.
        
            for i = 1:maxIterations
                % Generate a random sample in the joint space
                qRand = randConfig(robot);
        
                % Find nearest node in the tree
                [qNear, nearIdx] = findNearest(tree, qRand);
        
                % Steer from qNear towards qRand within the step size
                qNew = steerTowards(qNear, qRand, stepSize);
        
                % Check for collisions at qNew using existing collision checks
                if ~obj.collisionCheckSelf(robot, qNew) && ~obj.collisionGroundLPI(robot)
                    % Add qNew to the tree if no collision
                    tree = [tree; qNew];
                    parent = [parent, nearIdx];
        
                    % Check if the new configuration is close to the goal
                    if norm(qNew - qGoal) < stepSize
                        % Add qGoal to the tree and connect to the nearest node
                        tree = [tree; qGoal];
                        parent = [parent, size(tree, 1)];
                        break;
                    end
                end
            end
        
            % Extract the collision-free path from qStart to qGoal
            qPath = extractPath(tree, parent);
        end


    end
end
