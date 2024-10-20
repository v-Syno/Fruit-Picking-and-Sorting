classdef CollisionClass
    properties (Access = protected)
        ellipX;       % X-coordinates of ellipsoid surface
        ellipY;       % Y-coordinates of ellipsoid surface
        ellipZ;       % Z-coordinates of ellipsoid surface
        jointX;       % Ellipsoid semi-axis length in the X-direction
        jointY;       % Ellipsoid semi-axis length in the Y-direction
        linkLeng;     % Length of each link used for ellipsoid Z-axis
        JointEllipse; % Handles for the ellipsoid surfaces
        centreJoints; % Centers of ellipsoids along the robot's joints
    end
    
    methods
        function self = CollisionClass() 
            % Constructor for CollisionClass
            % Initialize properties
            self.JointEllipse = [];
            self.centreJoints = {};
        end
        
        function updateEllips(self, robot)
            % Update the ellipsoids to reflect the robot's current configuration
            % Clear any existing ellipsoids
            self.clearEllipsoids();
            
            % Check if the plyFileNameStem property exists
            if ~isprop(robot, 'plyFileNameStem')
                error('The robot does not have a "plyFileNameStem" property.');
            end
            
            % Get the current robot configuration
            tempAng = robot.model.getpos;
            [~, jointTr] = robot.model.fkine(tempAng);
            
            % Calculate the centers for each ellipsoid between joints
            numJoints = length(jointTr) - 1;
            self.centreJoints = cell(1, numJoints);
            for i = 2:length(jointTr)
                self.centreJoints{i - 1} = 0.5 * (jointTr(i - 1).t + jointTr(i).t);
            end
            
            % Set ellipsoid dimensions based on the robot type
            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                self.jointX = 0.058;
                self.jointY = 0.058;
                self.linkLeng = [0.15, 0.001, 0.11, 0.071, 0.095, 0.131, 0.095, 0.091, 0.051, 0.041, 0.001];
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3') || strcmp(robot.plyFileNameStem, 'LinearUR3e')
                self.jointX = 0.058;
                self.jointY = 0.058;
                self.linkLeng = [0.12, 0.158, 0.075, 0.031, 0.032, 0.0853, 0.031];
            else
                error('Unknown robot type: %s', robot.plyFileNameStem);
            end
            
            % Ensure the linkLeng array is the correct length
            if length(self.linkLeng) < numJoints
                error('The linkLeng array has fewer elements than the number of joints.');
            end
            
            % Generate ellipsoids for each joint center
            for j = 1:numJoints
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(...
                    self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3), ...
                    self.jointX, self.jointY, self.linkLeng(j));
                
                % Create a surface plot for the ellipsoid
                self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ, ...
                    'FaceAlpha', 0.3, 'EdgeColor', 'none');
            end
        end
        
        function outp = collisionCheck(self, point)
            % Check if a given point is within any ellipsoid
            outp = false;
            for i = 1:length(self.centreJoints)
                % Calculate the algebraic distance to the ellipsoid
                result = (point - self.centreJoints{i})' * ...
                         ([self.jointX^-2, 0, 0; 0, self.jointY^-2, 0; 0, 0, self.linkLeng(i)^-2]) * ...
                         (point - self.centreJoints{i});
                
                if result <= 1
                    % Collision detected
                    outp = true;
                    return;
                end
            end
        end
        
        function visualiseEllips(self)
            % Visualize the updated ellipsoids
            hold on;
            for j = 1:length(self.JointEllipse)
                % You can add more visualization configurations if needed
                rotate(self.JointEllipse(j), [0 0 1], 0); % No rotation needed, just a placeholder
            end
            hold off;
        end
        
        function clearEllipsoids(self)
            % Clear the displayed ellipsoids
            if ~isempty(self.JointEllipse)
                for j = 1:length(self.JointEllipse)
                    if isvalid(self.JointEllipse(j))
                        delete(self.JointEllipse(j));
                    end
                end
                self.JointEllipse = [];
            end
        end
    end
end
