clear;
clf;
clc;
close all;
set(0,'DefaultFigureWindowStyle','docked')
view(3)

hold on;

Objects = ObjectClass();
RobotControl = RobotClass();

axis([-1.8, 1.8, -1.8, 1.8, 0.01, 2]);

floor = imread ('grass.jpg'); % Floor Image

surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor

tomato = 'tomato.ply';

% tomatoes testing
tomatoTreePos = [
    -0.5, 1.12, 0.015;
    -0.35,1.16 0.015;
    0.5,1.1, 0.015;    
    ];
% use PlaceObjects NOT PlaceObjects2 as thats used for non moving
% components
[tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);

% unsorted box 
unsortedPos = [
    -0.4, 0.12, 0.015;
   -0.25, 0.12, 0.015;
   -0.1, 0.12, 0.015;
    ];

steps = 100;

% Initialize robots and their respective collision functions
harvesterBot = LinearUR3e(transl(0.4,0.6,0.01));

% Initialise gripper on UR3 end effector
gripperLength = 0.24;
gripperHeight = 0.025;

rightPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
leftPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
right = GripRight(rightPos); % initiate right gripper
left = GripLeft(leftPos); % initial left gripper

RobotControl.GripperMove(right,left,'open');

%% Harvesting 
% Neutral position to start from.
neutralPose = [0, 0.6, 0.4];
RobotControl.MoveRobot(harvesterBot, neutralPose, steps, [], [], false, 'forward',right,left);

% singularity

for i = 1:size(tomatoTreePos, 1)
    % Step 1: Move to the position of the tomato, slightly offset for approach.
    approachPose = tomatoTreePos(i, :) + [0,0,gripperHeight];
    RobotControl.MoveRobot(harvesterBot, approachPose, steps, [], [], false, 'down',right,left);

    % Step 2: Move directly above the tomato and then pick it up (close grippers).
    tomatoPickupPose = tomatoTreePos(i, :) + [0,0,gripperHeight];
    RobotControl.MoveRobot(harvesterBot, tomatoPickupPose, steps, tomatoObject{i}, tomatoVertices{i}, false, 'down',right,left);
    RobotControl.GripperMove(right, left, 'close'); % Close gripper to hold the tomato.

    % Step 3: Move the tomato to the unsorted crate, hover above the crate.
    hoverPose = unsortedPos(i, :) + [0, 0, 0.5];
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',right,left);

    % Step 4: Lower slightly into the crate.
    lowerPose = unsortedPos(i, :) + [0, 0, 0.25];
    RobotControl.MoveRobot(harvesterBot, lowerPose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down',right,left);

    % Step 5: Release the tomato by opening the gripper.
    RobotControl.GripperMove(right, left, 'open'); % Release the object.
    
    % Step 6: Move back to the hover position before transitioning to the next task.
    RobotControl.MoveRobot(harvesterBot, hoverPose, steps, [], [], false, 'down',right,left);
end

% Return to the neutral pose after completing the tasks.
RobotControl.MoveRobot(harvesterBot, neutralPose, steps, [], [], false, 'forward',right,left);