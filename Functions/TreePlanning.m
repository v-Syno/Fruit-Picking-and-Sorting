function [waymark_x,waymark_y,waymark_z] = TreePlanning(centerX,centerY,height,radius,numWaymarks)

    % Generate points for the circle plot
    theta = linspace(0, 2*pi, 100); % 100 points to make a smooth circle
    circle_x = centerX + radius * cos(theta);
    circle_y = centerY + radius * sin(theta);
    circle_z = height * ones(size(circle_x)); % Add height offset
    
    % Plot the circle on the floor
    plot3(circle_x, circle_y, circle_z, 'b-', 'LineWidth', 2); % Plot the circle in blue using 3D plot
    
    % Generate and plot waymarks at regular intervals
    waymark_theta = linspace(0, 2*pi, numWaymarks+1); % +1 to close the circle
    waymark_theta(end) = []; % Remove duplicate point at 2*pi
    waymark_x = centerX + radius * cos(waymark_theta);
    waymark_y = centerY + radius * sin(waymark_theta);
    waymark_z = height * ones(size(waymark_x)); % Add height offset for waymarks
    
    % Plot waymarks with red markers
    plot3(waymark_x, waymark_y, waymark_z, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Mark the center point (robot position)
    plot3(centerX, centerY, height, 'k+', 'MarkerSize', 12, 'LineWidth', 2);
    
    grid on;
    
    % Annotations to label waymarks
    for i = 1:numWaymarks
        text(waymark_x(i) + 0.05, waymark_y(i) + 0.05, sprintf('Waymark %d', i), 'FontSize', 8, 'Color', 'red');
    end
end