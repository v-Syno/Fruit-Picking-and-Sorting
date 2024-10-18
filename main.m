%% Assignment 2
% Daniel McMahon
% Jennifer Wilson

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)

Fruit = ObjectClass();
RobotControl = RobotClass();

%% Setup environment
clf;
clc;
view(3)
hold on

axis([-1.8, 1.8, -1.8, 1.8, 0.01, 2]);

floor = imread ('grass.jpg'); % Floor Image
wall = imread ('sky.PNG'); % wall Image

surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treeNormal.ply';
ObjectClass.PlaceObjects2(tree, [-1.15,1.25,0.01], 'Scale', [0.5, 0.5, 0.6], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(tree, [1.15,1.25,0.01], 'Scale', [0.5, 0.5, 0.6], 'Rotate', [0, 0, 0]);

% Boxes
appleCrate = 'apple_crate.ply';
orangeCrate = 'orange_crate.ply';
ObjectClass.PlaceObjects2(orangeCrate, [0,0.5,0.01], 'Scale', [1,1,1], 'Rotate', [0, 0, 0]); % TEMP storage create
ObjectClass.PlaceObjects2(appleCrate, [-0.45,-0.5,0.01], 'Scale', [1,1,1], 'Rotate', [0, 0, pi/2]); 
ObjectClass.PlaceObjects2(appleCrate, [0.45,-0.5,0.01], 'Scale', [1,1,1], 'Rotate', [0, 0, pi/2]);

% Cows

%% Safety Features

% Guard rails to secure the zone from external hazards
barrier = 'barrier.ply';
ObjectClass.PlaceObjects2(barrier, [1.2,-1.25,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(barrier, [-0.3,-1.25,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(barrier, [-1.8,-1.25,0], 'Scale', [1, 1, 0.5], 'Rotate', [0, 0, 0]);

% warning signs
sign1 = imread('sign1.png'); % robot sign
sign2 = imread('sign2.png'); % PPE sign

surf([0.7, 1.7; 0.7, 1.7], [-1.26, -1.26; -1.26, -1.26], [0.5, 0.5; 0.1, 0.1], 'CData', sign1, 'FaceColor', 'texturemap'); % Robot sign protection
surf([-0.75, 0.25; -0.75, 0.25], [-1.26, -1.26; -1.26, -1.26], [0.5, 0.5; 0.1, 0.1], 'CData', sign2, 'FaceColor', 'texturemap'); % PPE sign protection

%% Grow Fruits

% Define fruits
orange = 'orange.ply';
apple = 'orange.ply';

% oranges on tree positions
orangeTreePos = [
    -1.5, 0.7, 0.8;
    -1, 0.8, 0.9;
    -1.3, 1, 1.3;
    -0.9, 0.9, 1;    
    ];

% Use a loop to place all oranges
for i = 1:size(orangeTreePos, 1)
    [orangeObject, orangeVertices] = Fruit.PlaceObjects2(orange, orangeTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

% apples on tree positions
appleTreePos = [
    1, 0.7, 0.8;
    1, 0.8, 0.9;
    1.3, 1, 1.3;
    0.9, 0.9, 1;    
    ];

% Use a loop to place all apples
for i = 1:size(appleTreePos, 1)
    ObjectClass.PlaceObjects2(apple, appleTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

% unsorted box 
unsortedPos = [
    -0.3, 0.3, 0.08;
    0.2, 0.5, 0.08;
    -0.1, 0.4, 0.08;
    0.35, 0.55, 0.08;
    -0.05, 0.29, 0.08;
    0.1, 0.38, 0.08;
    -0.24, 0.34, 0.08;
    0.19, 0.49, 0.08
    ];

% oranges sorting box 
% x: -0.55 to -0.3
% y: -0.2 to -0.9
   orangeSorted = [
       -0.48, -0.25, 0.08;
       -0.35, -0.6, 0.08;
       -0.5, -0.4, 0.08;
       -0.4, -0.7, 0.08
       ];

% apples sorting box
appleSorted = [
     0.48, -0.3, 0.08;
     0.58, -0.4, 0.08;
     0.4, -0.45, 0.08;
     0.5, -0.25, 0.08
    ];

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
pandaRobot = Panda(transl(0,1,0.01));
ur3Robot = LinearUR3e(transl(0,0,0.01) * trotz(pi/2));

% % Initialise Gripper robots on UR3 end effector.
% pos1 = (ur3Robot.model.fkineUTS(ur3Robot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
% pos2 = (ur3Robot.model.fkineUTS(ur3Robot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
% g1 = GripRight(pos1); % initiate right gripper
% g2 = GripLeft(pos2); % initial left gripper
% RobotControl.GripperControl(g1, g2, 'close'); % Close Gripper to operating distance for Mandarin (open close 10 degrees)

pos3 = (pandaRobot.model.fkineUTS(pandaRobot.model.getpos()))* transl(0,-0.0127,0.05)*trotx(-pi/2)*trotz(pi);  % Base position right gripper offset from Panda's end effector 
pos4 = (pandaRobot.model.fkineUTS(pandaRobot.model.getpos()))* transl(0,0.0127,0.05)*trotx(-pi/2)*trotz(pi); % Base position left gripper offset from Panda's end effector 
g3 = GripRight(pos3); % initiate right gripper
g4 = GripLeft(pos4); % initial left gripper 

%% Tree harvesting - Orange

for i = 1:size(orangeTreePos, 1)
    % Step 1: Move to the position of the orange on the tree (approach from above)
    pickUpPosePanda = orangeTreePos(i, :) + [0, 0, 0.1];  % Position slightly above the orange
    RobotControl.MoveRobot(pandaRobot, pickUpPosePanda, steps, [], false, [], 'endEffDirection', 'down');

    % Step 2: Lower the end effector to the orange
    lowerPosePanda = orangeTreePos(i, :) + [0, 0, 0.05];  % Position to grip the orange
    RobotControl.MoveRobot(pandaRobot, lowerPosePanda, steps, [], false, [], 'endEffDirection', 'down');

    % Step 3: Close the gripper to grip the orange
    RobotControl.GripperMove(g3, g4, false); % Close the gripper around the orange to grip it

    % Step 4: Lift the orange back up slightly
    liftPosePanda = orangeTreePos(i, :) + [0, 0, 0.1];
    RobotControl.MoveRobot(pandaRobot, liftPosePanda, steps, [], true, orangeVertices{i});

    % Step 5: Move the orange to the unsorted box position
    unsortedPose = unsortedPos(i, :) + [0, 0, 0.1];  % Approach the unsorted box from above
    RobotControl.MoveRobot(pandaRobot, unsortedPose, steps, [], true, orangeVertices{i});

    % Step 6: Lower the orange into the unsorted box
    placePose = unsortedPos(i, :) + [0, 0, 0.05];
    RobotControl.MoveRobot(pandaRobot, placePose, steps, [], true, orangeVertices{i});

    % Step 7: Open the gripper to release the orange in the unsorted box
    RobotControl.GripperMove(g3, g4, true); % Open the gripper to release the orange

    % Step 8: Move back to a neutral position (above the unsorted box)
    neutralPosePanda = unsortedPos(i, :) + [0, 0, 0.3];
    RobotControl.MoveRobot(pandaRobot, neutralPosePanda, steps, [], false, []);
end