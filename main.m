%% Assignment 2
% Daniel McMahon
% Jennifer Wilson

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)
hold on

Objects = ObjectClass();
RobotControl = RobotClass();

% Setup Environment %

axis([-1.8, 1.8, -1.8, 1.8, 0.01, 2]);

floor = imread ('grass.jpg'); % Floor Image
soil = imread ('soil.jpg'); % Soil Image
wall = imread ('sky.PNG'); % wall Image

surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor
surf([-1.8,-1.8;1.8,1.8],[1,1.8;1,1.8],[0.01,0.01;0.01,0.02] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treeSkinnier.ply';
ObjectClass.PlaceObjects2(tree, [-0.7,1.4,0.01], 'Scale', [0.15,0.15,0.3]); % left tree (big)
ObjectClass.PlaceObjects2(tree, [0.7,1.2,0.01], 'Scale', [0.1, 0.1,0.2]); % right tree (small)

% Box
crate = 'crate.ply';
ObjectClass.PlaceObjects2(crate, [0.25,-0.85,0.55], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted


% table (raised surface for linearUR3e
slab = 'table.ply';
ObjectClass.PlaceObjects2(slab, [0,0.6,-0.2], 'Scale', [0.6,0.4,0.5]);
ObjectClass.PlaceObjects2(slab, [0.75,-0.25,-0.2], 'Scale', [0.2,0.2,0.5]);

% table
table = 'table.ply';
tableZ = 0.5;
ObjectClass.PlaceObjects2(table, [0,-0.95,0], 'Scale', [0.5,0.4,1]);

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

% Estop
estop = 'estop.ply';
ObjectClass.PlaceObjects2(estop, [-0.4,-1.1,tableZ], 'Scale', [0.5,0.5,0.5]);


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
    -0.6, 1.3, 0.32;
    -0.7, 1.25 0.4;
    0.75, 1.15, 0.16;    
    ];
% use PlaceObjects NOT PlaceObjects2 as thats used for non moving
% components
[tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);

% potatoes
potatoGroundPos = [
    -0.4, 1.11, 0.01;
    0.35, 1.38, 0.01;
    0.45, 1.2, 0.01;    
    ];

[potatoObject, potatoVertices] = Objects.PlaceObjects(potato, potatoGroundPos);

% Meeting point for both robots
meetPose = [-0.5,0,0.5];

% unsorted box 
boxPos = [
    0.4, -0.92, tableZ;
    0.25, -0.92, tableZ;
    0.1, -0.92, tableZ;
    0.4, -0.77, tableZ;
    0.25, -0.77, tableZ;
    0.1, -0.77, tableZ;
    ];

%% Task %%

steps = 100;

robotRaised = 0.05;

% Initialize robots and their respective collision functions
sortingBot = Panda(transl(0.75,-0.25,robotRaised) * trotz(pi));
harvesterBot = LinearUR3e(transl(0.4,0.6,robotRaised));

% Initialise gripper on UR3 end effector
gripperLength = 0.24;
gripperHeight = 0.025;

rightHarvesterPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
leftHarvesterPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
rightHarvester = GripRight(rightHarvesterPos); % initiate right gripper
leftHarvester = GripLeft(leftHarvesterPos); % initial left gripper

rightSorterPos = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))* transl(0,-0.0127,0.05)*trotx(-pi/2)*trotz(pi);  % Base position right gripper offset from Panda's end effector 
leftSorterPos = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))* transl(0,0.0127,0.05)*trotx(-pi/2)*trotz(pi); % Base position left gripper offset from Panda's end effector 
rightSorter = GripRight(rightSorterPos); % initiate right gripper
leftSorter = GripLeft(leftSorterPos); % initial left gripper 

RobotControl.GripperMove(rightHarvester,leftHarvester,'open');
RobotControl.GripperMove(rightSorter,leftSorter,'open');

%% Harvesting 
% Neutral position to start from.
neutralPose = [0, 0.6, 0.4];
RobotControl.MoveRobot(harvesterBot, neutralPose, steps, [], [], false, 'forward',rightHarvester,leftHarvester);


for i = 1:size(tomatoTreePos, 1)
    % Step 1: Move to the position of the tomato, slightly offset for approach.
    approachPose = tomatoTreePos(i, :);
    RobotControl.MoveRobot(harvesterBot, approachPose, steps, [], [], false, 'forward',rightHarvester,leftHarvester);

    % Step 2: Move directly to the tomato and then pick it up (close grippers).
    tomatoPickupPose = tomatoTreePos(i, :);
    RobotControl.MoveRobot(harvesterBot, tomatoPickupPose, steps, tomatoObject{i}, tomatoVertices{i}, false, 'forward',rightHarvester,leftHarvester);

    RobotControl.GripperMove(rightHarvester, leftHarvester, 'close'); % Close gripper to hold the tomato.

    % Step 3: Move the tomato to the unsorted crate, hover above the crate.
    hoverPose = boxPos(i, :) + [0, 0, 0.5];
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',rightHarvester,leftHarvester);

    % Step 4: Lower slightly into the crate.
    lowerPose = boxPos(i, :) + [0, 0, 0.25];
    RobotControl.MoveRobot(harvesterBot, lowerPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',rightHarvester,leftHarvester);

    % Step 5: Release the tomato by opening the gripper.
    RobotControl.GripperMove(rightHarvester, leftHarvester, 'open'); % Release the object.
    
    % Step 6: Move back to the hover position before transitioning to the next task.
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, [], [], false, 'down',rightHarvester,leftHarvester);
end

