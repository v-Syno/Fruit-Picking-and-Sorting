function waypoints = GenerateWaypoints(startPos, dropOffPos, gripperLength)
    % Adjusted y-position to maintain safe distance from obstacles.
    ySafe = 1 - gripperLength;
    
    % Define the waypoint path at y = ySafe to avoid the scarecrow.
    waypoint1 = [startPos(1), ySafe, startPos(3)]; % Move to ySafe directly above start.
    waypoint2 = [dropOffPos(1), ySafe, startPos(3)]; % Move along x to the drop-off x position.
    waypoint3 = [dropOffPos(1), dropOffPos(2), dropOffPos(3)]; % Move down along y to the drop-off position.
    
    % Define the reverse path to the next tomato tree.
    waypoint4 = [dropOffPos(1), ySafe, startPos(3)]; % Move back to ySafe after drop-off.
    waypoint5 = [startPos(1), ySafe, startPos(3)]; % Move back along x towards the next tomato.
    
    % Combine all waypoints.
    waypoints = {waypoint1, waypoint2, waypoint3, waypoint4, waypoint5};
end

