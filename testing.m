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

[tomatoTreePos, unsortedBoxPos, rightBox, leftBox] = PlantLocations(tableZ);

% use PlaceObjects NOT PlaceObjects2 as that's used for non-moving objects
[tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);

%% Load Robots

steps = 50;
[harvesterBot, sortingBot, rightHarvester, leftHarvester, rightSorter, leftSorter] = LoadRobots();

% Orientation for the end-effector.
pointForwards = trotx(270, 'deg');
pointDown = trotx(180, 'deg');
pointTowardsBox = trotx(90, 'deg');

%% Simulation of Harvest and Sorting Bots Working Together

% Variable to track when the sorting bot should start.
itemsDroppedOff = 0;
sortingBotStart = false;

% Loop for the harvest bot to pick and place tomatoes.
for i = 1:size(tomatoTreePos, 1)
    % Harvest Bot actions: Picking up and placing tomatoes.
    startPos = tomatoTreePos(i, :) + [0, -0.1, 0.1]; % Approach position for tomato.
    pickupPos = tomatoTreePos(i, :) + [0, -gripperLength, gripperHeight]; % Picking position.
    dropOffPos = unsortedBoxPos(i, :) + [0, 0, 0.2]; % Drop-off hover position above the unsorted box.
    finalPos = unsortedBoxPos(i, :) + [0, 0, 0.01]; % Drop-off position in the box.

    % Step 1: Move to the tomato.
    RobotControl.MoveRobot(harvesterBot, startPos, steps, [], [], false, rightHarvester, leftHarvester, pointDown);
    RobotControl.MoveRobot(harvesterBot, pickupPos, steps, tomatoObject{i}, tomatoVertices{i}, false, rightHarvester, leftHarvester, pointDown);
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'close');

    % Step 2: Move to the unsorted box.
    RobotControl.MoveRobot(harvesterBot, dropOffPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointDown);
    RobotControl.MoveRobot(harvesterBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightHarvester, leftHarvester, pointDown);
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'open');
    ObjectClass.DropObject(harvesterBot, tomatoObject{i}, tomatoVertices{i}, finalPos(3));

    % Increment the drop-off counter after the object is dropped off.
    itemsDroppedOff = itemsDroppedOff + 1;

    % Check if the sorting bot should start.
    if itemsDroppedOff >= 2
        sortingBotStart = true;
    end

    % If the sorting bot is triggered, perform its actions.
    if sortingBotStart && i <= size(rightBox, 1)
        % Sorting Bot actions: Picking from unsorted box and placing.
        startSortPos = unsortedBoxPos(i, :) + [0, 0, gripperLength + 0.1]; % Hover above the item.
        pickupSortPos = unsortedBoxPos(i, :) + [0, 0, gripperLength]; % Lower to the item's position.
        dropSortPos = rightBox(min(i, size(rightBox, 1)), :) + [0, 0, gripperLength + 0.1]; % Hover above the right box.
        finalSortPos = rightBox(min(i, size(rightBox, 1)), :) + [0, 0, gripperLength]; % Lower position for dropping.

        % Use the DualRobot function to coordinate the sorting process.
        RobotControl.DualRobot({harvesterBot, sortingBot}, {finalPos, pickupSortPos}, steps, ...
            {[], tomatoObject{i}}, {[], tomatoVertices{i}}, [false, true], ...
            {{rightHarvester, leftHarvester}, {rightSorter, leftSorter}}, {pointDown, pointDown});

        % Simulate the gripper closing to pick up the object.
        RobotControl.GripperMove(rightSorter, leftSorter, 'close');

        % Move to drop-off position.
        RobotControl.DualRobot({harvesterBot, sortingBot}, {finalPos, dropSortPos}, steps, ...
            {[], tomatoObject{i}}, {[], tomatoVertices{i}}, [false, true], ...
            {{rightHarvester, leftHarvester}, {rightSorter, leftSorter}}, {pointDown, pointDown});

        % Release the object in the right box.
        RobotControl.GripperMove(rightSorter, leftSorter, 'open');
        ObjectClass.DropObject(sortingBot, tomatoObject{i}, tomatoVertices{i}, finalSortPos(3));

        % Move the sorting bot back up.
        RobotControl.DualRobot({harvesterBot, sortingBot}, {finalPos, dropSortPos}, steps, ...
            {[], []}, {[], []}, [false, false], ...
            {{rightHarvester, leftHarvester}, {rightSorter, leftSorter}}, {pointDown, pointDown});
    end
end
