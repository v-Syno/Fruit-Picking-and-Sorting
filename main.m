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

% Use a loop to place all apples
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

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
pandaRobot = Panda();
ur3Robot = LinearUR3e();

%% Tree harvesting - Orange

for i = 1:size(orangeTreePos, 1)

    % go to orange on tree

    % open gripper and grip orange

    % Move orange to the unsorted box

    % place orange down, release gripper

end
