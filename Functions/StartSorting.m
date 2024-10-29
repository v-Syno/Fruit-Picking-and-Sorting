function StartSorting(app)

    % Initialize counters for each type of sorted produce.
    app.goodTomatoes = 0;
    app.badTomatoes = 0;
    app.goodPotatoes = 0;
    app.badPotatoes = 0;

    tomatoIndex = 1;
    potatoIndex = 1;

    % Initialize counters for the good and bad boxes.
    goodBoxCount = 0;
    badBoxCount = 0;
    
    numProduce = size(app.producePositions, 1);
    qualityLabels = ProduceClass.RandomizeQuality(numProduce);
    
    travelHeight = 1; % Set a reasonable height for traveling to avoid joint spasms.
    fixedXPos = 0.5; % Fixed X position during travel to ensure a more predictable path.
    
    idleSorter = [0.25, -0.2, 1.0]; % Idle pose;
    
    % Loop through each item in the unsorted box.
    for i = 1:numProduce
        % Determine the object type for each item and select the correct object and vertices.
        if strcmp(app.produceTags{i}, 'Tomatoes')
            % currentObject = app.tomatoObject{i};
            % currentVertices = app.tomatoVertices{i};
            currentObject = app.tomatoObject{tomatoIndex};
            currentVertices = app.tomatoVertices{tomatoIndex};
            tomatoIndex = tomatoIndex + 1;  % Increment tomato index only
        elseif strcmp(app.produceTags{i}, 'Potatoes')
            % currentObject = app.potatoObject{i};
            % currentVertices = app.potatoVertices{i};
            currentObject = app.potatoObject{potatoIndex};
            currentVertices = app.potatoVertices{potatoIndex};
            potatoIndex = potatoIndex + 1;  % Increment potato index only
        end
    
        % Define starting and hover positions above the item in the unsorted box.
        startPos = app.unsortedBox(i, :) + [0, 0, app.gripperLength + 0.5]; % Hover above the item.
        pickupItem = app.unsortedBox(i, :) + [0, 0, app.gripperLength]; % Lower to the item's position.
        hoverPos = [fixedXPos, pickupItem(2), travelHeight]; % Travel hover position with fixed X.
    
        % Determine the target box and EE pose based on the item's quality.
        if strcmp(qualityLabels{i}, 'good')
            targetBox = app.goodBox(min(goodBoxCount + 1, size(app.goodBox, 1)), :);
            targetEEPose = app.goodBoxEEPose;
            targetOrientation = app.pointBackwards; % Point towards negative Y for the good box.
            goodBoxCount = goodBoxCount + 1;

            % Update counters based on produce type.
            if strcmp(app.produceTags{i}, 'Tomatoes')
                app.goodTomatoes = app.goodTomatoes + 1;
            else
                app.goodPotatoes = app.goodPotatoes + 1;
            end
        else
            targetBox = app.badBox(min(badBoxCount + 1, size(app.badBox, 1)), :);
            targetEEPose = app.badBoxEEPose;
            targetOrientation = trotx(90, 'deg'); % Point towards positive X for the bad box.
            badBoxCount = badBoxCount + 1;

            % Update counters based on produce type.
            if strcmp(app.produceTags{i}, 'Tomatoes')
                app.badTomatoes = app.badTomatoes + 1;
            else
                app.badPotatoes = app.badPotatoes + 1;
            end
        end
    
        % Define the drop-off positions for the selected box.
        dropOffPos = [fixedXPos, targetEEPose(2), travelHeight]; % Travel position at fixed X and height.
        finalPos = targetEEPose; % Final EE hover position over the drop-off point.
    
        % Step 1: Move to hover above the unsorted item.
        RobotClass.MoveRobot(app.sortingBot, startPos, app.steps, [], [], false, app.rightSorter, app.leftSorter, app.pointDown);
    
        % Step 2: Lower to the item position (simulate pickup).
        RobotClass.MoveRobot(app.sortingBot, pickupItem, app.steps, currentObject, currentVertices, false, app.rightSorter, app.leftSorter, app.pointDown);
        RobotClass.GripperMove(app.rightSorter, app.leftSorter, 'close'); % Simulate gripping.
    
        % Step 3: Move back up to the hover position at travel height.
        RobotClass.MoveRobot(app.sortingBot, hoverPos, app.steps, currentObject, currentVertices, true, app.rightSorter, app.leftSorter, app.pointDown);
    
        % Step 4: Move in a straight line to the drop-off position.
        RobotClass.MoveRobot(app.sortingBot, dropOffPos, app.steps, currentObject, currentVertices, true, app.rightSorter, app.leftSorter, targetOrientation);
    
        % Step 5: Move to the final EE position above the target box.
        RobotClass.MoveRobot(app.sortingBot, finalPos, app.steps, currentObject, currentVertices, true, app.rightSorter, app.leftSorter, targetOrientation);
    
        % Step 6: Release the object.
        RobotClass.GripperMove(app.rightSorter, app.leftSorter, 'open');
        
        % Step 7: Place the item at the target box position.
        ObjectClass.DropObject(app.sortingBot, currentObject, currentVertices, targetBox);
    
        % Step 8: Move back up to a safe hover position after releasing the object.
        RobotClass.MoveRobot(app.sortingBot, finalPos + [0, 0, 0.3], app.steps, [], [], false, app.rightSorter, app.leftSorter, app.pointBackwards);
    
        % Step 9: Move to idle position after sorting.
        RobotClass.MoveRobot(app.sortingBot, idleSorter, app.steps, [], [], false, app.rightSorter, app.leftSorter, app.pointDown);
    end
    fprintf('\nSorting Summary:\n');
    fprintf('%d good tomatoes\n', app.goodTomatoes);
    fprintf('%d bad tomatoes\n', app.badTomatoes);
    fprintf('%d good potatoes\n', app.goodPotatoes);
    fprintf('%d bad potatoes\n', app.badPotatoes);

end