function StartHarvesting(app)

    for i = 1:size(app.producePositions, 1)
    % Determine the EE orientation, pickup offset, and current produce object based on produce type.
    if strcmp(app.produceTags{i}, 'Tomatoes')
        EEDirection = app.pointForwards;
        pickupOffset = [0, -app.gripperLength, app.gripperHeight]; % Offset for tomatoes on the tree.
        currentObject = app.tomatoObject{i};
        currentVertices = app.tomatoVertices{i};
    elseif strcmp(app.produceTags{i}, 'Potatoes')
        EEDirection = app.pointDown;
        pickupOffset = [0, 0, app.gripperLength]; % Offset for potatoes on the ground.
        currentObject = app.potatoObject{i};
        currentVertices = app.potatoVertices{i};
    end

    idleHarvester = [0, 1.25, 0.6];

    % Define approach and pickup positions based on produce position.
    startPos = app.producePositions(i, :) + [0, -0.3, 0.1]; % Approach position.
    pickupPos = app.producePositions(i, :) + pickupOffset; % Exact pickup position.
    
    % Define the drop-off positions at the unsorted box.
    hoverPos = app.unsortedBoxEEPos + [0, 0, 0.5]; % Hover above the unsorted box.
    dropOffPos = app.unsortedBoxEEPos; % Drop-off position.

    % Step 1: Move to approach position near the produce.
    RobotClass.MoveRobot(app.harvesterBot, startPos, app.steps, [], [], false, app.rightHarvester, app.leftHarvester, EEDirection);

    % Step 2: Move directly to the produce and simulate pickup.
    RobotClass.MoveRobot(app.harvesterBot, pickupPos, app.steps, currentObject, currentVertices, false, app.rightHarvester, app.leftHarvester, EEDirection);
    RobotClass.GripperMove(app.rightHarvester, app.leftHarvester, 'close'); % Simulate gripping.

    % Step 3: Move back slightly after gripping.
    moveBackPos = pickupPos + [0, -0.1, 0]; % Move back a bit after pickup.
    RobotClass.MoveRobot(app.harvesterBot, moveBackPos, app.steps, currentObject, currentVertices, true, app.rightHarvester, app.leftHarvester, EEDirection);

    % Step 4: Move to hover above the unsorted box.
    RobotClass.MoveRobot(app.harvesterBot, hoverPos, app.steps, currentObject, currentVertices, true, app.rightHarvester, app.leftHarvester, app.pointBackwards);

    % Step 5: Lower to the drop-off position.
    RobotClass.MoveRobot(app.harvesterBot, dropOffPos, app.steps, currentObject, currentVertices, true, app.rightHarvester, app.leftHarvester, app.pointBackwards);

    % Step 6: Release the object and update its position in the environment.
    RobotClass.GripperMove(app.rightHarvester, app.leftHarvester, 'open');
    ObjectClass.DropObject(app.harvesterBot, currentObject, currentVertices, app.unsortedBox(i, :));

    % Step 7: Move back up to a safe hover position.
    safePos = dropOffPos + [0, 0, 0.2];
    RobotClass.MoveRobot(app.harvesterBot, safePos, app.steps, [], [], false, app.rightHarvester, app.leftHarvester, app.pointBackwards);

    % Step 8: Move to idle position after each item.
    RobotClass.MoveRobot(app.harvesterBot, idleHarvester, app.steps, [], [], false, app.rightHarvester, app.leftHarvester, app.pointForwards);
    end
end

