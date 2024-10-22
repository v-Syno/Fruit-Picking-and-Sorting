%% Assignment 2
% Daniel McMahon
% Jennifer Wilson

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)

Objects = ObjectClass();
RobotControl = RobotClass();

%% Setup environment
clf;
clc;
view(3)
hold on

axis([-1.8, 1.8, -1.8, 1.8, 0.01, 2]);

floor = imread ('grass.jpg'); % Floor Image
soil = imread ('soil.jpg'); % Soil Image
wall = imread ('sky.PNG'); % wall Image

surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor
surf([-1.8,-1.8;1.8,1.8],[1,1.8;1,1.8],[0.01,0.01;0.01,0.02] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treeSkinnier.ply';
ObjectClass.PlaceObjects2(tree, [-0.5,1.4,0.01], 'Scale', [0.2,0.2,0.15]); % left tree (big)
ObjectClass.PlaceObjects2(tree, [0.5,1.2,0.01], 'Scale', [0.1, 0.1,0.1]); % right tree (small)

% Boxes
crate = 'crate.ply';
fruitCrate = crate;
vegCrate = crate;
ObjectClass.PlaceObjects2(crate, [-0.25,0.05,0.06], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted
ObjectClass.PlaceObjects2(fruitCrate, [-0.25,-0.5,0.06], 'Scale', [0.5,1,0.5]); % tomato
ObjectClass.PlaceObjects2(vegCrate, [0.5,-1.1,0.06], 'Scale', [0.5,1,0.5]); % potato

% table (raised surface for linearUR3e
slab = 'table.ply';
ObjectClass.PlaceObjects2(slab, [0,0.6,-0.2], 'Scale', [0.6,0.4,0.5]);

% Safety Features

% Guard rails to secure the zone from external hazards
barrier = 'barrier.ply';
ObjectClass.PlaceObjects2(barrier, [1.2,-1.5,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(barrier, [-0.3,-1.5,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(barrier, [-1.8,-1.5,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);

% warning signs
sign1 = imread('sign1.png'); % robot sign
sign2 = imread('sign2.png'); % PPE sign

surf([0.7, 1.7; 0.7, 1.7], [-1.56, -1.56; -1.56, -1.56], [0.5, 0.5; 0.1, 0.1], 'CData', sign1, 'FaceColor', 'texturemap'); % Robot sign protection
surf([-0.75, 0.25; -0.75, 0.25], [-1.51, -1.51; -1.51, -1.51], [0.5, 0.5; 0.1, 0.1], 'CData', sign2, 'FaceColor', 'texturemap'); % PPE sign protection

%% Grow Fruits

% Define fruits
tomato = 'tomato.ply';
potato = 'potato.ply';

tomatoSize = [0.07,0.07,0.06];
potatoSize = [0.06,0.11,0.05];

crateBase = 0.06;
tomatoMidPt = 0.03;
potatoMidPt = 0.025;

% tomatoes on trees
tomatoTreePos = [
    -0.5, 1.25, 0.15;
    -0.35,1.3 0.2;
    0.5,1.1, 0.09;    
    ];
% use PlaceObjects NOT PlaceObjects2 as thats used for non moving
% components
[tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);

% potatoes
potatoGroundPos = [
    0, 1.2, 0.01;
    -0.2, 1.3, 0.01;
    0.2, 1.25, 0.01;    
    ];

[potatoObject, potatoVertices] = Objects.PlaceObjects(potato, potatoGroundPos);


% unsorted box 
        % update positions
unsortedPos = [
    -0.15, 0.12, 0.01;
    0, 0.12, 0.01;
    0.15, 0.12, 0.01;
    -0.15, -0.02, 0.01;
    0, -0.02, 0.01;
    0.15, -0.02, 0.01;
    ];

% tomatoes sorting box 
% add mid point to z value to account for size of fruit (no clipping)
tomatoSorted = [
   -0.25, -0.35, 0.015;
   -0.25, -0.5, 0.015;
   -0.25, -0.65, 0.015;
   ];

% potatoes sorting box
% add mid point to z value to account for size of fruit (no clipping)
potatoSorted = [
    0.5, -0.95, 0.03;
    0.5, -1.1, 0.03;
    0.5, -1.25, 0.03;
   ];

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 100;

HarvesterBotRaised = 0.05;

% Initialize robots and their respective collision functions
% sortingBot = Panda(transl(0.5,-0.3,0.01) * trotz(pi/2));
harvesterBot = LinearUR3e(transl(0.4,0.6,HarvesterBotRaised));

% Initialise gripper on UR3 end effector
gripperLength = 0.16;
gripperOffset = 0.02;

rightPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
leftPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
right = GripRight(rightPos); % initiate right gripper
left = GripLeft(leftPos); % initial left gripper

RobotControl.GripperMove(right,left,'open');

%% Harvesting 
% Neutral position to start from.
neutralPose = [0, 0.6, 0.4];
RobotControl.MoveRobot(harvesterBot, neutralPose, steps, [], [], false, 'forward',right,left);

for i = 1:size(tomatoTreePos, 1)
    % Step 1: Move to the position of the tomato, slightly offset for approach.
    approachPose = tomatoTreePos(i, :);
    RobotControl.MoveRobot(harvesterBot, approachPose, steps, [], [], false, 'forward',right,left);

    % Step 2: Move directly above the tomato and then pick it up (close grippers).
    tomatoPickupPose = tomatoTreePos(i, :);
    RobotControl.MoveRobot(harvesterBot, tomatoPickupPose, steps, tomatoObject{i}, tomatoVertices{i}, false, 'forward',right,left);
    RobotControl.GripperMove(right, left, 'close'); % Close gripper to hold the tomato.

    % Step 3: Move the tomato to the unsorted crate, hover above the crate.
    hoverPose = unsortedPos(i, :) + [0, 0, 0.5];
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',right,left);

    % Step 4: Lower slightly into the crate.
    lowerPose = unsortedPos(i, :);
    RobotControl.MoveRobot(harvesterBot, lowerPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',right,left);

    % Step 5: Release the tomato by opening the gripper.
    RobotControl.GripperMove(right, left, 'open'); % Release the object.
    
    % Step 6: Move back to the hover position before transitioning to the next task.
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, [], [], false, 'forward',right,left);
end

% Return to the neutral pose after completing the tasks.
RobotControl.MoveRobot(harvesterBot, neutralPose, steps, [], [], false, 'forward',right,left);

%% To do:

% have the grippe point down above just above the crate and 'drop' the
% object in. therefore when we are above, we release gripper and update the
% fruits position to be in the crate. helps avoid ground collision