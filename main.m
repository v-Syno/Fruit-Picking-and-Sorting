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
axis([-1.8, 1.8, -1.8, 1.8, 0.01, 2]);


Objects = ObjectClass();
RobotControl = RobotClass();

% Initialise global variables
tableZ = 0.5;
slabHeight = 0.05;
gripperLength = 0.24;
gripperHeight = 0.025;
boxOffsetZ = gripperLength + slabHeight;

% Setup Environment
LoadEnvironment();

%% Grow Plants

% Define objects
tomato = 'tomato.ply';
potato = 'potato.ply';

tomatoSize = [0.07,0.07,0.06];
potatoSize = [0.06,0.11,0.05];

[tomatoTreePos,unsortedBoxPos,rightBox,leftBox] = PlantLocations(tableZ);


% use PlaceObjects NOT PlaceObjects2 as thats used for non moving objects
[tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);


%% Load Robots

steps = 75;

[harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots();

% Orientation for the end-effector.
pointForwards = trotx(270, 'deg');
pointDown = trotx(180,'deg');
pointTowardsBox = trotx(90, 'deg');


%% Testing Harvester Bot Movements with Tomatoes

% Loop through each tomato for testing.
for i = 1:size(tomatoTreePos, 1)
    % Define positions for this loop.
    startPos = tomatoTreePos(i, :) + [0, -0.3, 0.1]; % Slight offset to approach the tomato.
    pickupTomato = tomatoTreePos(i, :) + [0, -gripperLength, gripperHeight]; % Position to pick up the tomato.
    unsortedBoxPosOffset = unsortedBoxPos(i, :) + [0, 0, 0.5]; % Hover position above the box.
    finalPos = unsortedBoxPos(i, :) + [0, 0, 0.2]; % Slightly lower over the box for drop-off.

    % Step 1: Move to the position near the tomato (approach).
    RobotControl.MoveRobot(harvesterBot, startPos, steps, [], [], false, rightHarvester, leftHarvester, pointForwards);

    % Step 2: Move directly to the tomato and simulate pickup.
    RobotControl.MoveRobot(harvesterBot, pickupTomato, steps, tomatoObject{i}, tomatoVertices{i}, false, rightHarvester, leftHarvester, pointForwards);
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'close'); % Simulate gripping.

    % Step 3: Move back slightly after gripping.
    moveBackPos = pickupTomato + [0, -0.05, 0]; % Move back 5 cm along the y-axis.
    RobotControl.MoveRobot(harvesterBot, moveBackPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointForwards);

    % Step 4: Rotate +180 degrees to align with the box.
    RobotControl.MoveRobot(harvesterBot, unsortedBoxPosOffset, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointTowardsBox);

    % Step 5: Lower down to the box.
    RobotControl.MoveRobot(harvesterBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointTowardsBox);

    % Step 6: Release the object and update the object position.
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'open');
    ObjectClass.DropObject(harvesterBot, tomatoObject{i}, tomatoVertices{i}, 0.05); % Adjust to slightly drop the object.

    % Step 7: Rotate back +180 degrees to face the next tomato.
    nextTomatoPos = tomatoTreePos(min(i+1, size(tomatoTreePos, 1)), :) + [0, -gripperLength, gripperHeight];
    RobotControl.MoveRobot(harvesterBot, nextTomatoPos, steps, [], [], false, rightHarvester, leftHarvester, pointForwards);
end

%% Testing Sorting Bot Movements with Tomatoes

% Loop through each item in the unsorted box for visualization.
for i = 1:size(unsortedBoxPos, 1)

    % Adjust the starting position to pick from the unsorted box.
    startPos = unsortedBoxPos(i, :) + [0, 0, gripperLength + 0.1]; % Hover above the item.
    pickupItem = unsortedBoxPos(i, :) + [0, 0, gripperLength]; % Lower to the item's position.
    
    % Define the target position in the sorted box area.
    dropOffPos = leftBox(min(i, size(leftBox, 1)), :) + [0, 0, gripperLength + 0.1]; % Hover above the OK box.
    finalPos = leftBox(min(i, size(leftBox, 1)), :) + [0, 0, gripperLength]; % Lower position for dropping.

    % Step 1: Move to hover above the unsorted item.
    RobotControl.MoveRobot(sortingBot, startPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Step 2: Lower to the item position (simulate pickup).
    RobotControl.MoveRobot(sortingBot, pickupItem, steps, tomatoObject{i}, tomatoVertices{i}, false, rightSorter, leftSorter, pointDown);
    RobotControl.GripperMove(rightSorter, leftSorter, 'close'); % Simulate gripping.

    % Step 3: Move back up after gripping.
    RobotControl.MoveRobot(sortingBot, startPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

    % Step 4: Move to hover above the target drop-off position.
    RobotControl.MoveRobot(sortingBot, dropOffPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

    % Step 5: Lower to the drop-off position.
    RobotControl.MoveRobot(sortingBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

    % Step 6: Release the object.
    RobotControl.GripperMove(rightSorter, leftSorter, 'open');
    ObjectClass.DropObject(sortingBot, tomatoObject{i}, tomatoVertices{i}, tableZ+0.06); % Adjust the object's position in the environment.

    % Step 7: Move back up to a safe hover position after releasing the object.
    RobotControl.MoveRobot(sortingBot, dropOffPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

    % Step 8: Move to the next item in the unsorted box.
    nextItemPos = unsortedBoxPos(min(i + 1, size(unsortedBoxPos, 1)), :) + [0, 0, gripperLength + 0.1]; % Hover above the next item.
    RobotControl.MoveRobot(sortingBot, nextItemPos, steps, [], [], false, rightSorter, leftSorter, pointDown);
end


