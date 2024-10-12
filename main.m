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

%% Place Robots

UR3Location = transl(0,0,0.01) * trotz(pi/2);
UR3 = LinearUR3e(UR3Location);

PandaLocation = transl(0,1,0.01) * trotz(pi/2);
Panda = Panda(PandaLocation);