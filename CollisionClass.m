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
            pointOnPlane = [0, 0, 0.01];  % Ground plane is at z = 0.01.

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
    end

    methods (Static)
        function result = IsIntersectionPointInsideRectangle(intersectP, rectVerts)
        % Ensure rectVerts contains exactly four vertices
        if size(rectVerts, 1) ~= 4
            error('rectVerts must contain exactly four vertices.');
        end
        
        % Define two edges of the rectangle
        edge1 = rectVerts(2, :) - rectVerts(1, :);
        edge2 = rectVerts(4, :) - rectVerts(1, :);
        
        % Vector from the first vertex to the intersection point
        vectorToPoint = intersectP - rectVerts(1, :);
        
        % Project the vector onto the edges
        dot1 = dot(vectorToPoint, edge1) / dot(edge1, edge1);
        dot2 = dot(vectorToPoint, edge2) / dot(edge2, edge2);
        
        % Check if the point is within the bounds of the rectangle
        if dot1 >= 0 && dot1 <= 1 && dot2 >= 0 && dot2 <= 1
            result = true;
        else
            result = false;
        end
    end

        function result = IsCollisionWithPrism(robot, q, faces, vertex, faceNormals)
        % ISCOLLISIONWITHPRISM Check for collisions between the robot
        % and the prism.
        % Input:
        %   robot - SerialLink robot model
        %   q - Joint configuration (1xN array)
        %   faces, vertex, faceNormals - Mesh data for the prism
        % Output:
        %   result - Boolean indicating whether a collision is detected
        
        result = false;
        
        % Get the transforms of all joints
        tr = CollisionClass.GetLinkPoses(q, robot);
        
        % Check for intersections between robot links and prism faces
        for i = 1 : size(tr, 3) - 1
            for faceIndex = 1:size(faces, 1)
                vertOnPlane = vertex(faces(faceIndex, 1)', :);
                [intersectP, check] = CollisionClass.LinePlaneIntersection(...
                    faceNormals(faceIndex, :), vertOnPlane, ...
                    tr(1:3, 4, i)', tr(1:3, 4, i + 1)');
                
                % If there's a valid intersection, check if it's inside the rectangle
                if check == 1 && CollisionClass.IsIntersectionPointInsideRectangle(...
                        intersectP, vertex(faces(faceIndex, :)', :))
                    % Collision detected
                    result = true;
                    return; % Exit early if a collision is found
                end
            end
        end
    end


    function result = IsCollision(robot, q, faces, vertex, faceNormals)
        % ISCOLLISION Check for collisions along a given robot configuration
        % Input:
        %   robot - SerialLink robot model
        %   q - Joint configuration (1xN array)
        %   faces, vertex, faceNormals - Mesh data for the obstacle
        % Output:
        %   result - Boolean indicating whether a collision is detected
        
        result = false;
        
        % Get the transform of every joint
        tr = CollisionClass.GetLinkPoses(q, robot); % Updated to CollisionClass
        
        % Go through each link and each triangle face to check for intersections
        for i = 1 : size(tr, 3) - 1
            for faceIndex = 1:size(faces, 1)
                vertOnPlane = vertex(faces(faceIndex, 1)', :);
                [intersectP, check] = CollisionClass.LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, ...
                                        tr(1:3, 4, i)', tr(1:3, 4, i + 1)'); % Updated to CollisionClass
                
                % If using triangles, include an intersection point check
                % This can be removed if not using triangular faces
                if check == 1 && CollisionClass.IsPointInRectangle(intersectP, vertex(faces(faceIndex, :)', :)) % Updated to CollisionClass
                    result = true;
                    return; % Early exit if a collision is detected
                end
            end
        end
    end

    function [transforms] = GetLinkPoses(q, robot)
    % GETLINKPOSES Get the position of each link in the robot
    % Input:
    %   q - Joint configuration
    %   robot - SerialLink robot model
    % Output:
    %   transforms - 4x4xN matrix of transformation matrices for each link

    links = robot.links;
    numJoints = length(links);
    
    % Check if q has the same number of elements as the number of joints
    if length(q) ~= numJoints
        error('The number of joint angles in q must match the number of robot links.');
    end
    
    % Initialize the transforms array
    transforms = zeros(4, 4, numJoints + 1);
    transforms(:, :, 1) = robot.base;
    
    for i = 1:numJoints
        L = links(1, i);
        currentTransform = transforms(:, :, i);
        currentTransform = currentTransform * trotz(q(1, i) + L.offset) * ...
                           transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
        transforms(:, :, i + 1) = currentTransform;
    end
end


    function [intersectionPoint, check] = LinePlaneIntersection(planeNormal, pointOnPlane, linePoint1, linePoint2)
        % LINEPLANEINTERSECTION Find the intersection of a line and a plane
        % Input:
        %   planeNormal - Normal vector of the plane
        %   pointOnPlane - A point on the plane
        %   linePoint1, linePoint2 - Two points that define the line
        % Output:
        %   intersectionPoint - The intersection point
        %   check - 1 if the intersection occurs, 0 otherwise
        
        % Vector from linePoint1 to linePoint2
        lineVec = linePoint2 - linePoint1;
        
        % Calculate dot product of line direction and plane normal
        dotProduct = dot(planeNormal, lineVec);
        
        if abs(dotProduct) < 1e-6
            check = 0; % The line is parallel to the plane
            intersectionPoint = [0, 0, 0];
            return;
        end
        
        % Calculate the intersection point
        t = dot(planeNormal, (pointOnPlane - linePoint1)) / dotProduct;
        intersectionPoint = linePoint1 + t * lineVec;
        
        % Check if intersection is within the segment
        if t >= 0 && t <= 1
            check = 1;
        else
            check = 0;
        end
    end

    function result = IsPointInRectangle(point, rectangleVerts)
        % ISPOINTINRECTANGLE Check if a point is within a rectangular region
        % Input:
        %   point - The intersection point (1x3 vector)
        %   rectangleVerts - The four vertices of the rectangular face (4x3 matrix)
        % Output:
        %   result - Boolean indicating if the point is within the rectangle
        
        % Calculate the bounds of the rectangle in the x and y directions
        minX = min(rectangleVerts(:, 1));
        maxX = max(rectangleVerts(:, 1));
        minY = min(rectangleVerts(:, 2));
        maxY = max(rectangleVerts(:, 2));
        
        % Check if the point lies within these bounds
        if (point(1) >= minX && point(1) <= maxX) && (point(2) >= minY && point(2) <= maxY)
            result = true;
        else
            result = false;
        end
    end
end

end
