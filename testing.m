%% Assignment 2
% Daniel McMahon 14317958
% Jennifer Wilson 13936277

clear;
clf;
clc;
close all;
% set(0,'DefaultFigureWindowStyle','docked')
view(3)
hold on
axis([-2.5, 2.5, -2.5, 2.5, 0.01, 2]);

% Setup Environment
LoadEnvironment();

% Load Robots
steps = 5;
gripperLength = 0.24;
gripperHeight = 0.025;

% Orientation for the end-effector.
pointForwards = trotx(270, 'deg');
pointDown = trotx(180,'deg');
pointBackwards = trotx(90, 'deg');

[harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots();

%% Generate Produce
% unsorted box position for EE
unsortedBoxEEPos = [0, 0, 0.6]; % box to drop produce

% Sorted box positions for EE
goodBoxEEPose = [0.6, -1.75, 0.6];
badBoxEEPose = [1.1, -1, 0.6];

% locations for fruit to be placed when dropped
[unsortedBox,goodBox,badBox] = ProduceClass.BoxLocations();

% Tomatoes
[producePositions,tomatoObject,tomatoVertices,produceTags] = ProduceClass.GenerateTomatoes();

% Potatoes
% [producePositions,potatoObject,potatoVertices,produceTags] = ProduceClass.GeneratePotatoes();

% Mix
% [producePositions,potatoObject,potatoVertices,tomatoObject,tomatoVertices,produceTags] = ProduceClass.GenerateMix();


%% Testing Harvester Bot
steps = 15;

tomatoIndex = 1;
potatoIndex = 1;

% Loop through each produce position and associated tag.
for i = 1:size(producePositions, 1)
    % Determine the EE orientation, pickup offset, and current produce object based on produce type.
    if strcmp(produceTags{i}, 'Tomatoes')
        EEDirection = pointForwards;
        pickupOffset = [0, -gripperLength, gripperHeight]; % Offset for tomatoes on the tree.
        currentObject = tomatoObject{tomatoIndex};
        currentVertices = tomatoVertices{tomatoIndex};
        tomatoIndex = tomatoIndex + 1;  % Increment tomato index only
    elseif strcmp(produceTags{i}, 'Potatoes')
        EEDirection = pointDown;
        pickupOffset = [0, 0, gripperLength]; % Offset for potatoes on the ground.
        currentObject = potatoObject{potatoIndex};
        currentVertices = potatoVertices{potatoIndex};
        potatoIndex = potatoIndex + 1;  % Increment potato index only
    end

    idleHarvester = [0,1.25,0.6];

    % Define approach and pickup positions based on produce position.
    startPos = producePositions(i, :) + [0, -0.3, 0.3]; % Approach position.
    pickupPos = producePositions(i, :) + pickupOffset; % Exact pickup position.
    
    % Define the drop-off positions at the unsorted box.
    hoverPos = unsortedBoxEEPos + [0, 0, 0.5]; % Hover above the unsorted box.
    dropOffPos = unsortedBoxEEPos; % Drop-off position.

    % Step 1: Move to approach position near the produce.
    RobotClass.MoveRobotRMRC(harvesterBot, startPos, steps, [], [], false, rightHarvester, leftHarvester, EEDirection);

    % Step 2: Move directly to the produce and simulate pickup.
    RobotClass.MoveRobotRMRC(harvesterBot, pickupPos, steps, currentObject, currentVertices, false, rightHarvester, leftHarvester, EEDirection);
    RobotClass.GripperMove(rightHarvester, leftHarvester, 'close'); % Simulate gripping.

    % Step 3: Move back slightly after gripping.
    moveBackPos = pickupPos + [0, -0.1, 0]; % Move back a bit after pickup.
    RobotClass.MoveRobotRMRC(harvesterBot, moveBackPos, steps, currentObject, currentVertices, true, rightHarvester, leftHarvester, EEDirection);

    % Step 4: Move to hover above the unsorted box.
    RobotClass.MoveRobotRMRC(harvesterBot, hoverPos, steps, currentObject, currentVertices, true, rightHarvester, leftHarvester, pointBackwards);

    % Step 5: Lower to the drop-off position.
    RobotClass.MoveRobotRMRC(harvesterBot, dropOffPos, steps, currentObject, currentVertices, true, rightHarvester, leftHarvester, pointBackwards);

    % Step 6: Release the object and update its position in the environment.
    RobotClass.GripperMove(rightHarvester, leftHarvester, 'open');
    ObjectClass.DropObject(harvesterBot, currentObject, currentVertices, unsortedBox(i, :));

    % Step 7: Move back up to a safe hover position.
    safePos = dropOffPos + [0, 0, 0.2];
    RobotClass.MoveRobotRMRC(harvesterBot, safePos, steps, [], [], false, rightHarvester, leftHarvester, pointBackwards);

end
    RobotClass.MoveRobotRMRC(harvesterBot, idleHarvester, steps, [], [], false, rightHarvester, leftHarvester, pointForwards);



%% Testing Sorting Bot

% Initialize counters for each type of sorted produce.
goodTomatoes = 0;
badTomatoes = 0;
goodPotatoes = 0;
badPotatoes = 0;

tomatoIndex = 1;
potatoIndex = 1;

% Initialize counters for the good and bad boxes.
goodBoxCount = 0;
badBoxCount = 0;

numProduce = size(producePositions, 1);
qualityLabels = ProduceClass.RandomizeQuality(numProduce);

travelHeight = 1; % Set a reasonable height for traveling to avoid joint spasms.
fixedXPos = 0.25; % Fixed X position during travel to ensure a more predictable path.

idleSorter = [0.25, -0.2, 1.0]; % Idle pose;

% Loop through each item in the unsorted box.
for i = 1:numProduce
    % Determine the object type for each item and select the correct object and vertices.
    if strcmp(produceTags{i}, 'Tomatoes')
        currentObject = tomatoObject{tomatoIndex};
        currentVertices = tomatoVertices{tomatoIndex};
        tomatoIndex = tomatoIndex + 1;  % Increment tomato index only
    elseif strcmp(produceTags{i}, 'Potatoes')
        currentObject = potatoObject{potatoIndex};
        currentVertices = potatoVertices{potatoIndex};
        potatoIndex = potatoIndex + 1;  % Increment potato index only
    end

    % Define starting and hover positions above the item in the unsorted box.
    startPos = unsortedBox(i, :) + [0, 0, gripperLength + 0.5]; % Hover above the item.
    pickupItem = unsortedBox(i, :) + [0, 0, gripperLength]; % Lower to the item's position.
    hoverPos = [fixedXPos, pickupItem(2), travelHeight]; % Travel hover position with fixed X.

    % Determine the target box and EE pose based on the item's quality.
    if strcmp(qualityLabels{i}, 'good')
        targetBox = goodBox(min(goodBoxCount + 1, size(goodBox, 1)), :);
        targetEEPose = goodBoxEEPose;
        targetOrientation = pointBackwards;
        goodBoxCount = goodBoxCount + 1;

        % Update counters based on produce type.
        if strcmp(produceTags{i}, 'Tomatoes')
            goodTomatoes = goodTomatoes + 1;
        else
            goodPotatoes = goodPotatoes + 1;
        end
    else
        targetBox = badBox(min(badBoxCount + 1, size(badBox, 1)), :);
        targetEEPose = badBoxEEPose;
        targetOrientation = pointBackwards;
        badBoxCount = badBoxCount + 1;

        % Update counters based on produce type.
        if strcmp(produceTags{i}, 'Tomatoes')
            badTomatoes = badTomatoes + 1;
        else
            badPotatoes = badPotatoes + 1;
        end
    end

    % Define the drop-off positions for the selected box.
    dropOffPos = [fixedXPos, targetEEPose(2), travelHeight]; % Travel position at fixed X and height.
    finalPos = targetEEPose; % Final EE hover position over the drop-off point.

    % Step 1: Move to hover above the unsorted item.
    RobotClass.MoveRobotRMRC(sortingBot, startPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Step 2: Lower to the item position (simulate pickup).
    RobotClass.MoveRobotRMRC(sortingBot, pickupItem, steps, currentObject, currentVertices, false, rightSorter, leftSorter, pointDown);
    RobotClass.GripperMove(rightSorter, leftSorter, 'close'); % Simulate gripping.

    % Step 3: Move back up to the hover position at travel height.
    RobotClass.MoveRobotRMRC(sortingBot, hoverPos, steps, currentObject, currentVertices, true, rightSorter, leftSorter, pointDown);

    % Step 4: Move in a straight line to the drop-off position.
    RobotClass.MoveRobotRMRC(sortingBot, dropOffPos, steps, currentObject, currentVertices, true, rightSorter, leftSorter, targetOrientation);

    % Step 5: Move to the final EE position above the target box.
    RobotClass.MoveRobotRMRC(sortingBot, finalPos, steps, currentObject, currentVertices, true, rightSorter, leftSorter, targetOrientation);

    % Step 6: Release the object.
    RobotClass.GripperMove(rightSorter, leftSorter, 'open');
    
    % Step 7: Place the item at the target box position.
    ObjectClass.DropObject(sortingBot, currentObject, currentVertices, targetBox);

    % Step 8: Move back up to a safe hover position after releasing the object.
    RobotClass.MoveRobotRMRC(sortingBot, finalPos + [0, 0, 0.3], steps, [], [], false, rightSorter, leftSorter, pointBackwards);

    % Step 9: Move to idle position after sorting.
    RobotClass.MoveRobotRMRC(sortingBot, idleSorter, steps, [], [], false, rightSorter, leftSorter, pointDown);
end

fprintf('\nSorting Summary:\n');
fprintf('%d good tomatoes\n', goodTomatoes);
fprintf('%d bad tomatoes\n', badTomatoes);
fprintf('%d good potatoes\n', goodPotatoes);
fprintf('%d bad potatoes\n', badPotatoes);