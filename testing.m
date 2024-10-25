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

% locations for fruit to be placed when dropped
[unsortedBox,goodBox,badBox] = Produce.BoxLocations();

% Generate Produce
[tomatoTreePos,tomatoObject,tomatoVertices] = Produce.GenerateTomatoes();

% [potatoGroundPos,potatoObject,potatoVertices] = Produce.GeneratePotatoes();

% testing unsortedBox
% [testObject, testVert] = Object.PlaceObjects(potato, unsortedBox);


%% Load Robots

steps = 50;

[harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots();

% Orientation for the end-effector.
pointPosY = trotx(270, 'deg');
pointDown = trotx(180,'deg');
pointNegY = trotx(90, 'deg');


%% Loop through each tomato
for i = 1:size(tomatoTreePos, 1)
    % Define positions for this loop.
    idlePos = [0, 1.25, 0.6];
    startPos = tomatoTreePos(i, :) + [0, -0.3, 0.1]; % Slight offset to approach the tomato.
    pickupTomato = tomatoTreePos(i, :) + [0, -gripperLength, gripperHeight]; % Position to pick up the tomato.
    unsortedBoxPosOffset = unsortedBoxEEPos + [0, 0, 0.5]; % Hover position above the box.
    finalPos = unsortedBoxEEPos; % Lower over the box for drop-off.

    % Step 1: Move to the position near the tomato (approach).
    RobotControl.MoveRobot(harvesterBot, startPos, steps, [], [], false, rightHarvester, leftHarvester, pointPosY);

    % Step 2: Move directly to the tomato and simulate pickup.
    RobotControl.MoveRobot(harvesterBot, pickupTomato, steps, tomatoObject{i}, tomatoVertices{i}, false, rightHarvester, leftHarvester, pointPosY);
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'close'); % Simulate gripping.

    % Step 3: Lift the EE up after gripping to avoid obstacles.
    liftedPos = pickupTomato + [0, 0, 0.3]; % Raise by 0.3 meters.
    RobotControl.MoveRobot(harvesterBot, liftedPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointPosY);

    % Step 4: Ensure the EE stays within a radius of 0.5 from the pickup position.
    % Adjust the path to follow a circular arc if needed.
    distanceToCenter = norm(liftedPos(1:2) - tomatoTreePos(i, 1:2));
    if distanceToCenter > 0.5
        % Calculate a new intermediate position to maintain the radius.
        direction = (liftedPos(1:2) - tomatoTreePos(i, 1:2)) / distanceToCenter;
        constrainedPos = tomatoTreePos(i, 1:2) + 0.5 * direction;
        constrainedPos = [constrainedPos, liftedPos(3)]; % Keep the Z height from liftedPos.
        RobotControl.MoveRobot(harvesterBot, constrainedPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointPosY);
    else
        constrainedPos = liftedPos;
    end

    % Step 5: Move to hover position above the unsorted box.
    RobotControl.MoveRobot(harvesterBot, unsortedBoxPosOffset, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointNegY);

    % Step 6: Lower down to the box.
    RobotControl.MoveRobot(harvesterBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointNegY);

    % Step 7: Release the object and update the object position.
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'open');
    ObjectClass.DropObject(harvesterBot, tomatoObject{i}, tomatoVertices{i}, unsortedBox(i, :)); % Adjust placement in unsorted box.

    % Step 8: Move up and back a bit to avoid collisions.
    finalPosAdjust = finalPos + [0, 0.1, 0.2]; % Lift up and move slightly back.
    RobotControl.MoveRobot(harvesterBot, finalPosAdjust, steps, [], [], false, rightHarvester, leftHarvester, pointNegY);
end

% Return the harvester bot to an idle position.
RobotControl.MoveRobot(harvesterBot, idlePos, steps, [], [], false, rightHarvester, leftHarvester, pointPosY);


%% Testing Sorting Bot Movements with Tomatoes

SortTomatoes(sortingBot, unsortedBox, goodBox, badBox, tomatoObject, tomatoVertices, rightSorter, leftSorter);

