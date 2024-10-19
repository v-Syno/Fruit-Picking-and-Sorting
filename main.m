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
soil = imread ('soil.jpg'); % Soil Image
wall = imread ('sky.PNG'); % wall Image

surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor
surf([-1.8,-1.8;1.8,1.8],[0.8,1.5;0.8,1.5],[0.01,0.01;0.01,0.02] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treetiny.ply';
ObjectClass.PlaceObjects2(tree, [-0.5,1,0.01], 'Scale', [3.5,3.5,1.5]); % left tree (big)
ObjectClass.PlaceObjects2(tree, [0.5,0.8,0.01], 'Scale', [2, 2, 0.6]); % right tree (small)

% Boxes
crate = 'crate.ply';
fruitCrate = crate;
vegCrate = crate;
ObjectClass.PlaceObjects2(crate, [0,0.05,0.05], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted
ObjectClass.PlaceObjects2(fruitCrate, [-0.5,-0.5,0.05], 'Scale', [0.5,1,0.5]); % fruit
ObjectClass.PlaceObjects2(vegCrate, [0.5,-0.5,0.05], 'Scale', [0.5,1,0.5]);% veg

% add image of fruit/veg on each sorted crate

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
tomato = 'tomato.ply';
potato = 'potato.ply';

% oranges (place holder for tomatoes) on tree positions
tomatoTreePos = [
    -0.5, 0.75, 0.3;
    -0.4,0.7 0.25;
    0.5,0.7, 0.09;    
    ];

% Use a loop to place all oranges
for i = 1:size(tomatoTreePos, 1)
    [orangeObject, orangeVertices] = Fruit.PlaceObjects2(tomato, tomatoTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

% apples(place holder for potatoes) on tree positions
potatoTreePos = [
    0, 1, 0.03;
    -0.2, 0.9, 0.03;
    0.2, 1.3, 0.03;    
    ];

% Use a loop to place all apples
for i = 1:size(potatoTreePos, 1)
    ObjectClass.PlaceObjects2(potato, potatoTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

% unsorted box 
unsortedPos = [
    -0.15, 0.12, 0.015;
    0, 0.12, 0.015;
    0.15, 0.12, 0.015;
    -0.15, -0.02, 0.015;
    0, -0.02, 0.015;
    0.15, -0.02, 0.015;
    ];

% oranges sorting box 
% x: -0.55 to -0.3
% y: -0.2 to -0.9
orangeSorted = [
   -0.5, -0.35, 0.015;
   -0.5, -0.5, 0.015;
   -0.5, -0.65, 0.015;
   ];

% apples sorting box
appleSorted = [
   0.5, -0.35, 0.015;
   0.5, -0.5, 0.015;
   0.5, -0.65, 0.015;
   ];

%% Testing

tomato = 'tomato.ply';
potato = 'potato.ply';

tomatoPosTest = [
%     -0.4, 1, 0.01
%     -0.6, 0.8, 0.01
%     ];
% [
    -0.15, 0.12, 0.015;
    0, 0.12, 0.015;
    0.15, 0.12, 0.015;
    -0.15, -0.02, 0.015;
    0, -0.02, 0.015;
    0.15, -0.02, 0.015;
    ];

% potatoPosTest = [
%     -0.3, 0.3, 0.01;
%     0.2, 0.5, 0.01;
%     ];

for i = 1:size(tomatoPosTest, 1)
    [orangeObject, orangeVertices] = Fruit.PlaceObjects2(tomato, tomatoPosTest(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

for i = 1:size(potatoPosTest, 1)
    [orangeObject, orangeVertices] = Fruit.PlaceObjects2(potato, potatoPosTest(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
pandaRobot = Panda(transl(0,-0.5,0.01) * trotz(pi/2));
ur3Robot = LinearUR3e(transl(0.3,0.5,0.01) );

%% Tree harvesting - Orange

for i = 1:size(tomatoPosTest, 1)
    % Step 1: Move to the position of the orange on the tree
    poseOrange = tomatoPosTest(i, :);
    RobotClass.MoveObject(pandaRobot, poseOrange, steps, orangeObject{i}, orangeVertices{i}, false,1);

    % Step 2: Lower the end effector to the orange

    % Step 3: Close the gripper to grip the orange

    % Step 4: Lift the orange back up slightly

    % Step 5: Move the orange to the unsorted box position

    % Step 6: Lower the orange into the unsorted box

    % Step 8: Move back to a neutral position (above the unsorted box)

end