%% Assignment 2
% Daniel McMahon 14317958
% Jennifer Wilson 13936277

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)
hold on
axis([-2.5, 2.5, -2.5, 2.5, 0.01, 2]);


Object = ObjectClass();
RobotControl = RobotClass();
Produce = ProduceClass();

% Setup Environment
LoadEnvironment();

% Load Robots

steps = 50;
gripperLength = 0.24;
gripperHeight = 0.025;

% Orientation for the end-effector.
pointPosY = trotx(270, 'deg');
pointDown = trotx(180,'deg');
pointNegY = trotx(90, 'deg');
pointNegX = trotz(90,'deg');
pointPosX = trotz(270,'deg');

[harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots();

%% Generate Produce

producePositions = []; % Initialize empty array for positions
produceTags = {};      % Initialize empty cell array for tags

% unsorted box position for EE
unsortedBoxEEPos = [0, 0, 0.6]; % box to drop produce

% Sorted box positions for EE
goodBoxEEPose = [0.6, -1.75, 0.6];
badBoxEEPose = [1.1, -1, 0.6];

% locations for fruit to be placed when dropped
[unsortedBoxMix,unsortedBox,goodBox,badBox] = Produce.BoxLocations();

% Tomatoes
[tomatoTreePos,tomatoObject,tomatoVertices] = Produce.GenerateTomatoes();

% Potatoes
% [potatoGroundPos,potatoObject,potatoVertices] = Produce.GeneratePotatoes();

% Mix
% [potatoGroundPos,potatoObject,potatoVertices,tomatoTreePos,tomatoObject,tomatoVertices] = GenerateMix()


%% Testing Harvester Bot Movements with Tomatoes

% Loop through each tomato
for i = 1:size(tomatoTreePos, 1)
    % Define positions for this loop.
    idleHarvester = [0,1.25,0.6];
    startPos = tomatoTreePos(i, :) + [0, -0.3, 0.1]; % Slight offset to approach the tomato.
    pickupTomato = tomatoTreePos(i, :) + [0, -gripperLength, gripperHeight]; % Position to pick up the tomato.
    unsortedBoxPosOffset = unsortedBoxEEPos + [0, 0, 0.5]; % Hover position above the box.
    finalPos = unsortedBoxEEPos; % lower over the box for drop-off.

    % Step 1: Move to the position near the tomato (approach).
    RobotControl.MoveRobot(harvesterBot, startPos, steps, [], [], false, rightHarvester, leftHarvester, pointPosY);

    % Step 2: Move directly to the tomato and simulate pickup.
    RobotControl.MoveRobot(harvesterBot, pickupTomato, steps, tomatoObject{i}, tomatoVertices{i}, false, rightHarvester, leftHarvester, pointPosY);
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'close'); % Simulate gripping.

    % Step 3: Move back slightly after gripping.
    moveBackPos = pickupTomato + [0, -0.1, 0]; % Move back
    RobotControl.MoveRobot(harvesterBot, moveBackPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointPosY);

    % Step 4: Move to hover position above the unsorted box.
    RobotControl.MoveRobot(harvesterBot, unsortedBoxPosOffset, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointNegY);

    % Step 5: Lower down to the box.
    RobotControl.MoveRobot(harvesterBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointNegY);

    % Step 6: Release the object and update the object position.
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'open');
    ObjectClass.DropObject(harvesterBot, tomatoObject{i}, tomatoVertices{i}, unsortedBox(i, :)); % Adjust placement in unsorted box

    % Step 7: move up and back a bit
    finalPosAdjust = finalPos + [0,0.1,0.2];
    RobotControl.MoveRobot(harvesterBot, finalPosAdjust, steps, [], [], false, rightHarvester, leftHarvester, pointNegY);

    RobotControl.MoveRobot(harvesterBot, idleHarvester, steps, [], [], false, rightHarvester, leftHarvester, pointPosY);
end


%% Testing Sorting Bot Movements with Tomatoes

% Initialize counters for the good and bad boxes.
goodBoxCount = 0;
badBoxCount = 0;

numProduce = size(unsortedBox, 1);
qualityLabels = Produce.RandomizeQuality(numProduce);

travelHeight = 0.8; % Set a reasonable height for traveling to avoid joint spasms.
fixedXPos = 0.25; % Fixed X position during travel to ensure a more predictable path.

idleSorter = [0.25, -0.2, 1.0]; % Idle pose;

% Loop through each item in the unsorted box for visualization.
for i = 1:size(unsortedBox, 1)
    % Define positions for this loop.
    startPos = unsortedBox(i, :) + [0, 0, gripperLength + 0.5]; % Hover above the item.
    pickupItem = unsortedBox(i, :) + [0, 0, gripperLength]; % Lower to the item's position.
    hoverPos = [fixedXPos, pickupItem(2), travelHeight]; % Hover position after pickup at fixed X.

    % Determine the target box and EE pose based on the quality.
    if strcmp(qualityLabels{i}, 'good')
        targetBox = goodBox(min(goodBoxCount + 1, size(goodBox, 1)), :);
        targetEEPose = goodBoxEEPose;
        targetOrientation = pointNegY; % Point towards negative Y for the good box.
        goodBoxCount = goodBoxCount + 1;
    else
        targetBox = badBox(min(badBoxCount + 1, size(badBox, 1)), :);
        targetEEPose = badBoxEEPose;
        targetOrientation = trotx(90, 'deg'); % Point towards positive X for the bad box.
        badBoxCount = badBoxCount + 1;
    end

    % Define the drop-off positions for the selected box.
    dropOffPos = [fixedXPos, targetEEPose(2), travelHeight]; % Travel position at fixed X and height.
    finalPos = targetEEPose; % Hover position for the EE over the drop-off point.

    % Step 1: Move to hover above the unsorted item.
    RobotControl.MoveRobot(sortingBot, startPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Step 2: Lower to the item position (simulate pickup).
    RobotControl.MoveRobot(sortingBot, pickupItem, steps, tomatoObject{i}, tomatoVertices{i}, false, rightSorter, leftSorter, pointDown);
    RobotControl.GripperMove(rightSorter, leftSorter, 'close'); % Simulate gripping.

    % Step 3: Move back up to the hover position at travel height.
    RobotControl.MoveRobot(sortingBot, hoverPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

    % Step 4: Move in a straight line to the drop-off position.
    % Maintain the same X position and height to avoid unnecessary rotations.
    RobotControl.MoveRobot(sortingBot, dropOffPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, targetOrientation);

    % Step 5: Move to the final EE position above the target box.
    RobotControl.MoveRobot(sortingBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, targetOrientation);

    % Step 6: Release the object.
    RobotControl.GripperMove(rightSorter, leftSorter, 'open');
    
    % Step 7: Use the DropObject function to place the tomato in the correct position.
    ObjectClass.DropObject(sortingBot, tomatoObject{i}, tomatoVertices{i}, targetBox);

    % Step 8: Move back up to a safe hover position after releasing the object.
    RobotControl.MoveRobot(sortingBot, finalPos + [0, 0, 0.3], steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Move to idle position after sorting.
    RobotControl.MoveRobot(sortingBot, idleSorter, steps, [], [], false, rightSorter, leftSorter, pointDown);
end