classdef CollisionClass
    properties (Access = protected)
        ellipX; % X-coordinates for ellipsoid surface
        ellipY; % Y-coordinates for ellipsoid surface
        ellipZ; % Z-coordinates for ellipsoid surface
        jointX; % Radius along X-axis for ellipsoid
        jointY; % Radius along Y-axis for ellipsoid
        linkLength; % Length of each robot link
        ellipsoidSurfaces; % Handles for plotted ellipsoids
        linkCenters; % Centers of the ellipsoids
    end
    
    methods
        % Constructor to initialize the class
        function self = CollisionClass(robot)
            self.initializeEllipsoids(robot);
        end
        
        % Initialize ellipsoids based on the robot configuration
        function initializeEllipsoids(self, robot)
            % Set default radii for ellipsoids (can be modified as needed)
            self.jointX = 0.1; % Example value for X-radius
            self.jointY = 0.1; % Example value for Y-radius
            self.linkLength = 0.3; % Example value for link length
            
            % Initialize ellipsoids based on the robot's current position
            self.updateEllipsoids(robot);
        end
        
        % Update ellipsoid positions and orientations based on the robot's current configuration
        function updateEllipsoids(self, robot)
            % Get current joint positions for the robot
            jointAngles = robot.model.getpos(); 
            
            % Calculate forward kinematics using the joint angles with fkineUTS
            % Assuming 'fkineUTS' returns an array of 4x4 transformation matrices
            jointTransforms = robot.model.fkineUTS(jointAngles);
            
            % Calculate the centers of each link (midpoint between consecutive joints)
            numLinks = size(jointTransforms, 3) - 1;
            self.linkCenters = cell(numLinks, 1);
            for i = 1:numLinks
                % Midpoint between the i-th and (i+1)-th joint
                % Extract translation components from the transformation matrices
                t1 = jointTransforms(1:3, 4, i);
                t2 = jointTransforms(1:3, 4, i+1);
                self.linkCenters{i} = 0.5 * (t1 + t2);
            end
            
            % Visualize the updated ellipsoids
            self.visualizeEllipsoids(robot, 0.5); % Update visualization
        end
        
        % Visualize the ellipsoids around each link
        function visualizeEllipsoids(self, robot, duration)
            % Delete old ellipsoid surfaces if they exist
            if ~isempty(self.ellipsoidSurfaces)
                for i = 1:length(self.ellipsoidSurfaces)
                    if isvalid(self.ellipsoidSurfaces{i})
                        delete(self.ellipsoidSurfaces{i});
                    end
                end
            end
            
            % Get the current joint transformations using fkineUTS
            jointAngles = robot.model.getpos(); 
            jointTransforms = robot.model.fkineUTS(jointAngles);
            numLinks = size(jointTransforms, 3) - 1;
            self.ellipsoidSurfaces = cell(numLinks, 1);
            
            % Generate and plot ellipsoids for each link
            for j = 1:numLinks
                % Get the link center
                center = self.linkCenters{j};
                % Generate ellipsoid data points
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(center(1), center(2), center(3), ...
                    self.jointX, self.jointY, self.linkLength / 2);
                
                % Plot the ellipsoid
                self.ellipsoidSurfaces{j} = surf(self.ellipX, self.ellipY, self.ellipZ);
                alpha(self.ellipsoidSurfaces{j}, 0.3); % Set transparency
                
                % Rotate ellipsoid to match the link orientation
                % Extract rotation angles from the transformation matrix
                rotationAngles = rad2deg(tr2rpy(jointTransforms(:,:,j)));
                rotate(self.ellipsoidSurfaces{j}, [0 0 1], rotationAngles(3), center);
                rotate(self.ellipsoidSurfaces{j}, [0 1 0], rotationAngles(2), center);
                rotate(self.ellipsoidSurfaces{j}, [1 0 0], rotationAngles(1), center);
            end
            
            % Pause for visualization
            pause(duration);
        end
        
        % Check if points lie inside any of the ellipsoids
        function collisionDetected = checkCollision(self, points)
            collisionDetected = false;
            for j = 1:length(self.linkCenters)
                % Calculate algebraic distance for each point
                algebraicDist = GetAlgebraicDist(points, self.linkCenters{j}, ...
                    [self.jointX, self.jointY, self.linkLength / 2]);
                
                % Check if any points are inside the ellipsoid
                if any(algebraicDist < 1)
                    collisionDetected = true;
                    disp(['Collision detected with link ', num2str(j)]);
                    break;
                end
            end
        end
    end
end

% Helper function to calculate algebraic distance
function dist = GetAlgebraicDist(points, center, radii)
    % points: Nx3 matrix of [x, y, z] coordinates to check
    % center: 1x3 vector of ellipsoid center [x_c, y_c, z_c]
    % radii: 1x3 vector of ellipsoid radii [r_x, r_y, r_z]

    % Calculate the algebraic distance for each point
    dist = ((points(:,1) - center(1)).^2 / radii(1)^2) + ...
           ((points(:,2) - center(2)).^2 / radii(2)^2) + ...
           ((points(:,3) - center(3)).^2 / radii(3)^2);
end
