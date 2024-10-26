%% Objects Class

classdef ObjectClass
    % Object class holds functions for all things related to objects
    %   Detailed explanation goes here

    methods (Static)

        function [sizeX, sizeY, sizeZ] = GetObjectSize(plyFile)
            % getObjectSize Reads a PLY file and calculates the object's dimensions.
            %
            %   [sizeX, sizeY, sizeZ] = getObjectSize('filename.ply') reads the PLY file,
            %   displays the point cloud, and returns the size along the x, y, and z axes.
            %
            % Inputs:
            %   - plyFile: String, name of the PLY file to read.
            %
            % Outputs:
            %   - sizeX: Length of the object along the x-axis.
            %   - sizeY: Length of the object along the y-axis.
            %   - sizeZ: Length of the object along the z-axis.
        
            % Read the point cloud data
            ptCloud = pcread(plyFile);
            
            % Display the point cloud
            pcshow(ptCloud);
            title('3D Object Point Cloud');
            xlabel('X'); ylabel('Y'); zlabel('Z');
            
            % Get the limits of the point cloud
            xLimits = ptCloud.XLimits;
            yLimits = ptCloud.YLimits;
            zLimits = ptCloud.ZLimits;
            
            % Calculate the size of the object
            sizeX = diff(xLimits);
            sizeY = diff(yLimits);
            sizeZ = diff(zLimits);
            
            % Display the size
            fprintf('Size along X-axis: %.2f\n', sizeX);
            fprintf('Size along Y-axis: %.2f\n', sizeY);
            fprintf('Size along Z-axis: %.2f\n', sizeZ);
        end
        
        function [objects, vertices] = PlaceObjects(objectName,objectLocation)
            % Place objects given their location, number of objects and
            % vertices

            % Initialize output cell arrays
            objects = cell(size(objectLocation,1),1);
            vertices = cell(size(objectLocation,1),1);

            for i = 1:size(objectLocation,1)
                x = objectLocation(i,1);
                y = objectLocation(i,2);
                z = objectLocation(i,3);

                objects{i} = PlaceObject(objectName);
                vertices{i} = get(objects{i}, 'Vertices');

                % Compute the transformation. transpose for multiplication
                verts = [vertices{i}, ones(size(vertices{i}, 1), 1)] * transl([x,y,z])';

                % Update the object's position and orientation
                set(objects{i}, 'Vertices', verts(:, 1:3));
            end
        end
        
        function [objects, vertices] = PlaceObjects2(objectName, objectLocation, options)
        % Place objects and adjust scaling and rotation
        
            arguments
                objectName {mustBeText}
                objectLocation (:,3) double
                options.Scale (1,3) double = [1, 1, 1]  % Default scaling factors for x, y, z
                options.Rotate (1,3) double = [0, 0, 0] % Rotation angles in degrees
            end
        
            % Initialize output cell arrays
            numObjects = size(objectLocation, 1);
            objects = cell(numObjects, 1);
            vertices = cell(numObjects, 1);
        
            for i = 1:numObjects
                x = objectLocation(i, 1);
                y = objectLocation(i, 2);
                z = objectLocation(i, 3);
        
                % Place the object and retrieve vertices
                objects{i} = PlaceObject(objectName);
                verts = get(objects{i}, 'Vertices');
        
                % Apply scaling to each axis directly
                verts(:, 1) = verts(:, 1) * options.Scale(1);  % Scale along x-axis
                verts(:, 2) = verts(:, 2) * options.Scale(2);  % Scale along y-axis
                verts(:, 3) = verts(:, 3) * options.Scale(3);  % Scale along z-axis
        
                % Create the rotation matrices using trotx, troty, and trotz
                T_rot = trotx(options.Rotate(1)) * troty(options.Rotate(2)) * trotz(options.Rotate(3));
        
                % Homogeneous transformation: [verts ones] for 4x4 transformation
                vertsHomogeneous = [verts, ones(size(verts, 1), 1)] * T_rot';
        
                % Translation: move object to specified location
                T_trans = transl([x, y, z]);
                vertsTransformed = vertsHomogeneous * T_trans';
        
                % Update vertices for the object (strip homogeneous part)
                vertices{i} = vertsTransformed(:, 1:3);
        
                % Set the new vertices to the object
                set(objects{i}, 'Vertices', vertices{i});
            end
        end
    
        function UpdateObjectPosition(robot, qMatrix, object, vertices)
            % Compute the transformation matrix for the end-effector.
            transMatrix = robot.model.fkineUTS(qMatrix) * transl(0,0,0.2);
            
            % Transform object vertices.
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transMatrix';
            set(object, 'Vertices', transformedVertices(:, 1:3));
        end

        function DropObject(robot, object, vertices, targetPosition)
            % robot: The robot model controlling the object.
            % object: The object being placed.
            % vertices: The vertices of the object.
            % targetPosition: [x, y, z] target position for the object drop.
        
            % Get the current joint configuration of the robot.
            qCurr = robot.model.getpos();
            poseCurr = robot.model.fkineUTS(qCurr);
        
            % Adjust the position based on the provided targetPosition.
            poseCurr(1, 4) = targetPosition(1); % X position.
            poseCurr(2, 4) = targetPosition(2); % Y position.
            poseCurr(3, 4) = targetPosition(3); % Z position.
        
            % Transform object vertices to the new position.
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * poseCurr';
            set(object, 'Vertices', transformedVertices(:, 1:3));
        end

    end
end