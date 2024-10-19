function [sizeX, sizeY, sizeZ] = getObjectSize(plyFile)
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


