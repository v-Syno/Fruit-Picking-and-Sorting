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
surf([-1.8,-1.8;1.8,1.8],[0.8,1.5;0.8,1.5],[0.01,0.01;0.01,0.02] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treeSkinnier.ply';
ObjectClass.PlaceObjects2(tree, [-0.5,1.1,0.01], 'Scale', [0.2,0.2,0.15]); % left tree (big)
ObjectClass.PlaceObjects2(tree, [0.5,1,0.01], 'Scale', [0.1, 0.1,0.1]); % right tree (small)

% Boxes
crate = 'crate.ply';
fruitCrate = crate;
vegCrate = crate;
ObjectClass.PlaceObjects2(crate, [0,0.05,0.05], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted
ObjectClass.PlaceObjects2(fruitCrate, [-0.5,-0.5,0.05], 'Scale', [0.5,1,0.5]); % fruit
ObjectClass.PlaceObjects2(vegCrate, [0.5,-0.5,0.05], 'Scale', [0.5,1,0.5]);% veg

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
tomato = 'tomato.ply';
potato = 'potato.ply';

tomatoSize = [0.07,0.07,0.06];
potatoSize = [0.06,0.11,0.05];

% tomatoes on trees
tomatoTreePos = [
    -0.5, 0.9, 0.15;
    -0.35,1.1 0.2;
    0.5,0.95, 0.09;    
    ];

[tomatoObject, tomatoVertices] = Objects.PlaceObjects2(tomato, tomatoTreePos, 'Scale', [1.5, 1.5, 1.5]);

% potatoes
potatoTreePos = [
    0, 1, 0.01;
    -0.2, 0.9, 0.01;
    0.2, 1.3, 0.01;    
    ];

[potatoObject, potatoVertices] = Objects.PlaceObjects2(potato, potatoTreePos, 'Scale', [1.5, 1.5, 1.5]);


% unsorted box 
unsortedPos = [
    -0.15, 0.12, 0.015;
    0, 0.12, 0.015;
    0.15, 0.12, 0.015;
    % -0.15, -0.02, 0.015; %uncomment later, just testing
    % 0, -0.02, 0.015;
    % 0.15, -0.02, 0.015;
    ];

% tomatoes sorting box 
tomatoSorted = [
   -0.5, -0.35, 0.015;
   -0.5, -0.5, 0.015;
   -0.5, -0.65, 0.015;
   ];

% potatoes sorting box
potatoSorted = [
   0.5, -0.35, 0.015;
   0.5, -0.5, 0.015;
   0.5, -0.65, 0.015;
   ];

%% Setup obstacles
cat = 'cat.ply';
catPos = [0,0,0.01];
[catObject, catVertices] = Objects.PlaceObjects2(cat, catPos, 'Scale', [1,1,1]);

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
%sortingBot = Panda(transl(0,-0.5,0.01) * trotz(pi/2));
harvesterBot = LinearUR3e(transl(0.3,0.5,0.01) );

%% Harvesting Tomatos

for i = 1:size(tomatoTreePos, 1)
    % Step 1: Move to the position of tomato on the tree. Give room for
    % gripper
    tomatoPose = tomatoTreePos(i, :);
    RobotClass.MoveObject2(harvesterBot, tomatoPose, steps, tomatoObject{i}, tomatoVertices{i}, false);

    % Step 2: Close the gripper to grip the tomato
    tomatoPose = tomatoTreePos(i, :);
    RobotClass.MoveObject2(harvesterBot, tomatoPose, steps, tomatoObject{i}, tomatoVertices{i}, true);

    % Step 2: move tomato to the unsorted crate pose
    unsortedCratePose = unsortedPos(i, :) + [0,0,0.1];
    RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, true);

    % Step 4: move EE pose to point down (z axis down) and lower tomato in
    % crate
    unsortedCratePose = unsortedPos(i, :);
    RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, true);

    % Step 5: release gripper
    unsortedCratePose = unsortedPos(i, :);
    RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, false);    

end