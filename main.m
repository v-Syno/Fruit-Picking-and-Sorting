%% Assignment 2
% Daniel McMahon
% Jennifer Wilson

clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)

FruitnVeg = ObjectClass();
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
surf([-1.1,-1.1;1.1,1.1],[0.8,1.5;0.8,1.5],[0.011,0.011;0.011,0.011] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)

% Trees
tree = 'treetiny.ply';
ObjectClass.PlaceObjects2(tree, [-0.9,1.25,0.01], 'Scale', [1,1,2.5]);
ObjectClass.PlaceObjects2(tree, [0.9,1.25,0.01], 'Scale', [0.5, 0.5, 0.6]);

% Boxes
crate = 'crate.ply';
fruitCrate = crate;
vegCrate = crate;
ObjectClass.PlaceObjects2(crate, [0,0.5,0.05], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]);
ObjectClass.PlaceObjects2(fruitCrate, [-0.45,-0.5,0.05], 'Scale', [0.5,1,0.5]); 
ObjectClass.PlaceObjects2(vegCrate, [0.45,-0.5,0.05], 'Scale', [0.5,1,0.5]);

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

% Define fruit and veg
tomato = 'tomato.ply';
potato = 'potato.ply';

% oranges on tree positions
orangeTreePos = [
    -1.5, 0.7, 0.8;
    -1, 0.8, 0.9;
    -1.3, 1, 1.3;
    -0.9, 0.9, 1;    
    ];

% Use a loop to place all oranges
for i = 1:size(orangeTreePos, 1)
    [orangeObject, orangeVertices] = FruitnVeg.PlaceObjects2(tomato, orangeTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
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
    ObjectClass.PlaceObjects2(potato, appleTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
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

%% Testing

tomato = 'potato.ply';

orangeTreePosTest = [
    -0.6, 0.8, 0.8
    -0.6, 0.8, 0.9
    ];

unsortedPosTest = [
    -0.3, 0.3, 0.08;
    0.2, 0.5, 0.08;
    ];

for i = 1:size(orangeTreePosTest, 1)
    [orangeObject, orangeVertices] = FruitnVeg.PlaceObjects2(tomato, orangeTreePosTest(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
pandaRobot = Panda(transl(0.5,-0.25,0.01) * trotz(pi/2));
ur3Robot = LinearUR3e(transl(0.5,0.5,0.01));

%% Tree harvesting - Orange

for i = 1:size(orangeTreePosTest, 1)
    % Step 1: Move to the position of the orange on the tree
    poseOrange = orangeTreePosTest(i, :);
    RobotClass.MoveObject(pandaRobot, poseOrange, steps, orangeObject{i}, orangeVertices{i}, false,1);

    % Step 2: Lower the end effector to the orange

    % Step 3: Close the gripper to grip the orange

    % Step 4: Lift the orange back up slightly

    % Step 5: Move the orange to the unsorted box position

    % Step 6: Lower the orange into the unsorted box

    % Step 8: Move back to a neutral position (above the unsorted box)

end