%% Assignment 2
% Daniel McMahon
% Jennifer Wilson

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)


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
orangeFile = 'orange.ply';
appleFile = 'orange.ply';

% oranges on tree positions
orangeTreePos = [
    -1.5, 0.7, 0.8;
    -1, 0.8, 0.9;
    -1.3, 1, 1.3;
    -0.9, 0.9, 1;    
    ];

% Use a loop to place all oranges
for i = 1:size(orangeTreePos, 1)
    ObjectClass.PlaceObjects2(orangeFile, orangeTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

% apples on tree positions
appleTreePos = [
    1, 0.7, 0.8;
    1, 0.8, 0.9;
    1.3, 1, 1.3;
    0.9, 0.9, 1;    
    ];

% Use a loop to place all oranges
for i = 1:size(appleTreePos, 1)
    ObjectClass.PlaceObjects2(appleFile, appleTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
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

%% Place Robots
orange = 'orange.ply';
ObjectClass.PlaceObjects2(orange, [1,0.7,0.6], 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
ObjectClass.PlaceObjects2(orange, [0.1,0.4,0.05], 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);


%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
pandaRobot = RobotControl.Initialize('Panda');
ur3Robot = RobotControl.Initialize('UR3');
collF = CollisionFunctions();

% Neutral positions for both robots
neutralPosePanda = [0, 0, 0.01 + 0.3];
neutralPoseUR3 = [1, 1, 0.01 + 0.3];  % Adjust for UR3 starting location

% Move Panda and UR3 to neutral positions
pandaRobot.MoveRobot(neutralPosePanda, steps, [], false, [], 1, [], [], 0, []);
ur3Robot.MoveRobot(neutralPoseUR3, steps, [], false, [], 1, [], [], 0, []);

%% Loop through all fruit locations
tic
for i = 1:size(fruit_location, 1)

    %% Panda picks fruit from the tree and places it into the box
    
    % Move Panda to fruit location
    pickUpPosePanda = fruit_location(i,:) + [0, 0, fruitZ + 0.05];  % Approach fruit from above
    pandaRobot.MoveRobot(pickUpPosePanda, steps, [], false, [], 1, [], [], 0, []);

    % Pick up fruit (lower end effector slightly)
    pickUpPosePanda = fruit_location(i,:) + [0, 0, 0.05];
    pandaRobot.MoveRobot(pickUpPosePanda, steps, [], false, [], 1, [], [], 1, []);  % Close gripper to pick fruit

    % Move Panda to the box location and place the fruit
    boxPosePanda = box_location + [0, 0, fruitZ + 0.1];  % Approach box with some clearance
    pandaRobot.MoveRobot(boxPosePanda, steps, [], true, fruit_vertices{i}, 1, [], [], 1, []);  % Carry fruit to the box

    placePosePanda = box_location + [0, 0, fruitZ];  % Place fruit in box
    pandaRobot.MoveRobot(placePosePanda, steps, [], true, fruit_vertices{i}, 1, [], [], 1, []);  % Release fruit in the box
    
    % Open gripper
    pandaRobot.MoveRobot(placePosePanda, steps, [], false, [], 1, [], [], 2, []);  % Open gripper to release fruit

    %% UR3 picks fruit from the box and sorts it into the sorting box

    % Move UR3 to the box to pick up the fruit
    pickUpPoseUR3 = box_location + [0, 0, fruitZ + 0.05];
    ur3Robot.MoveRobot(pickUpPoseUR3, steps, [], false, [], 1, [], [], 0, []);

    % Pick up fruit from the box
    pickUpPoseUR3 = box_location + [0, 0, 0.05];
    ur3Robot.MoveRobot(pickUpPoseUR3, steps, [], false, [], 1, [], [], 1, []);  % Close gripper to pick fruit

    % Move UR3 to the sorting box and place the fruit
    sortingBoxPoseUR3 = sorting_box_location(i, :) + [0, 0, fruitZ + 0.1];  % Approach sorting box
    ur3Robot.MoveRobot(sortingBoxPoseUR3, steps, [], true, fruit_vertices{i}, 1, [], [], 1, []);  % Carry fruit to sorting box

    placePoseUR3 = sorting_box_location(i, :) + [0, 0, fruitZ];  % Place fruit in sorting box
    ur3Robot.MoveRobot(placePoseUR3, steps, [], true, fruit_vertices{i}, 1, [], [], 1, []);  % Release fruit in sorting box

    % Open gripper to release the fruit
    ur3Robot.MoveRobot(placePoseUR3, steps, [], false, [], 1, [], [], 2, []);  % Open gripper
    
    %% Log progress
    RobotControl.Logs(ur3Robot, sorting_box_location(i,:));

    elapsedTime = toc;
    progress = (i/size(fruit_location,1)) * 100;
    fprintf('\n Seconds: %.2f for Progress: %.2f%%\n', elapsedTime, progress);

end

% Return both robots to neutral positions after completion
pandaRobot.MoveRobot(neutralPosePanda, steps, [], false, [], 1, [], [], 0, []);
ur3Robot.MoveRobot(neutralPoseUR3, steps, [], false, [], 1, [], [], 0, []);

fprintf('\n Fruit sorting task complete \n');
