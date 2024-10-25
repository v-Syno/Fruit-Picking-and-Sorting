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

% Initialise global variables
tableZ = 0.4;
gripperLength = 0.24;
gripperHeight = 0.025;

% Setup Environment
LoadEnvironment();

%% Generate Produce

% unsorted box position for EE
unsortedBoxEEPos = [0, 0, tableZ+0.2]; % box to drop produce

% Sorted box positions for EE
goodBoxEEPose = [0.6, -1.75, tableZ+0.2];
badBoxEEPose = [0, 0, tableZ+0.2];

% locations for fruit to be placed when dropped
[unsortedBox,goodBox,badBox] = Produce.BoxLocations();

% Generate Produce
[tomatoTreePos,tomatoObject,tomatoVertices] = Produce.GenerateTomatoes();
[potatoGroundPos,potatoObject,potatoVertices] = Produce.GeneratePotatoes();

% testing unsortedBox
% [testObject, testVert] = Object.PlaceObjects(potato, unsortedBox);


%% Load Robots

steps = 50;

[harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots();

% Orientation for the end-effector.
pointPosY = trotx(270, 'deg');
pointDown = trotx(180,'deg');
pointNegY = trotx(90, 'deg');


%% Testing Harvester Bot Movements with Tomatoes

% Loop through each tomato
for i = 1:size(tomatoTreePos, 1)
    % Define positions for this loop.
    idlePos = [0,1.25,0.6];
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
end

RobotControl.MoveRobot(harvesterBot, idlePos, steps, [], [], false, rightHarvester, leftHarvester, pointPosY);

%% Testing Sorting Bot Movements with Tomatoes

% Loop through each item in the unsorted box for visualization.
for i = 1:size(unsortedBox, 1)

    % Adjust the starting position to pick from the unsorted box.
    startPos = unsortedBox(i, :) + [0, 0, gripperLength + 0.1]; % Hover above the item.
    pickupItem = unsortedBox(i, :) + [0, 0, gripperLength]; % Lower to the item's position.
    
    % Use `goodBoxEEPose` as the reference hover position for the good box.
    dropOffPos = goodBoxEEPose + [0, 0, 0.3]; % Hover above the good box.
    finalPos = goodBoxEEPose; % Final position for dropping the object.

    % Step 1: Move to hover above the unsorted item.
    RobotControl.MoveRobot(sortingBot, startPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Step 2: Lower to the item position (simulate pickup).
    RobotControl.MoveRobot(sortingBot, pickupItem, steps, tomatoObject{i}, tomatoVertices{i}, false, rightSorter, leftSorter, pointDown);
    RobotControl.GripperMove(rightSorter, leftSorter, 'close'); % Simulate gripping.

    % Step 3: Move back up after gripping.
    hoverPos = pickupItem + [0,0,0.8];
    RobotControl.MoveRobot(sortingBot, hoverPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

    % Step 4: Move to hover above the target drop-off position.
    RobotControl.MoveRobot(sortingBot, dropOffPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointNegY);

    % Step 5: Lower to the drop-off position.
    RobotControl.MoveRobot(sortingBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointNegY);

    % Step 6: Release the object.
    RobotControl.GripperMove(rightSorter, leftSorter, 'open');
    ObjectClass.DropObject(sortingBot, tomatoObject{i}, tomatoVertices{i}, goodBox(i,:)); % Adjust the object's position in the environment.

    % Step 7: Move back up to a safe hover position after releasing the object.
    RobotControl.MoveRobot(sortingBot, dropOffPos + [0,0,0.5], steps, [], [], false, rightSorter, leftSorter, pointDown);

end

