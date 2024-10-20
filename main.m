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

% collision testing
collisionHandler = CollisionClass();


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
ObjectClass.PlaceObjects2(crate, [0,0.3,0.06], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted
ObjectClass.PlaceObjects2(fruitCrate, [-0.25,-0.5,0.06], 'Scale', [0.5,1,0.5]); % tomato
ObjectClass.PlaceObjects2(vegCrate, [0.5,-1.1,0.06], 'Scale', [0.5,1,0.5]); % potato

%% Safety Features

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

%% testing
% tomatoTreePos = [
%    -0.25, -0.35, 0.015;
%    -0.25, -0.5, 0.015;
%    -0.25, -0.65, 0.015;    
%     ];
% 
% [tomatoObject, tomatoVertices] = Objects.PlaceObjects(tomato, tomatoTreePos);

%% Setup obstacles
cat = 'cat.ply';
catPos = [0,0,0.01];
[catObject, catVertices] = Objects.PlaceObjects(cat, catPos);

%% Fruit Sorting with Collision Detection for Panda and UR3

steps = 50;

% Initialize robots and their respective collision functions
sortingBot = Panda(transl(0.5,-0.3,0.01) * trotz(pi/2));
harvesterBot = LinearUR3e(transl(0.3,0.74,0.01) );

%%
% Initialise gripper on UR3 end effector.
rightPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
leftPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
right = GripRight(rightPos); % initiate right gripper
left = GripLeft(leftPos); % initial left gripper

RobotControl.GripperMove(right,left,'close'); % Close Gripper to operating distance for object (open close 10 degrees)


%% Harvesting Tomatos
% 
% for i = 1:size(tomatoTreePos, 1)
%     % Step 1: Move to the position of the tomato on the tree, with room for the gripper.
%     tomatoPose = tomatoTreePos(i, :) + [0, 0, 0.1]; % Offset to avoid collision initially
%     RobotClass.MoveRobot(harvesterBot, tomatoPose, steps, tomatoObject{i}, tomatoVertices{i}, false, 'forward', right, left);
% 
%     % Step 2: Move to the tomato position and pick it up.
%     RobotClass.MoveRobot(harvesterBot, tomatoTreePos(i, :), steps, tomatoObject{i}, tomatoVertices{i}, true, 'down', right, left);
% 
%     % Step 3: Move the tomato to the unsorted crate position.
%     unsortedCratePose = unsortedPos(i, :) + [0, 0, 0.1];
%     RobotClass.MoveRobot(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'forward', right, left);
% 
%     % Step 4: Lower the tomato into the crate.
%     unsortedCratePose = unsortedPos(i, :) + [0, 0, 0.05]; % Adjust based on tomato size
%     RobotClass.MoveRobot(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'down', right, left);
% 
%     % Step 5: Release the gripper.
%     RobotClass.MoveRobot(harvesterBot, unsortedPos(i, :), steps, tomatoObject{i}, tomatoVertices{i}, false, 'down', right, left);
% end

%% Harvesting Potatoes
% 
% for i = 1:size(potatoGroundPos, 1)
%     % Step 1: Move to the position of tomato on the tree. Give room for
%     % gripper
%     potatoPose = potatoGroundPos(i, :);
%     RobotClass.MoveObject2(harvesterBot, potatoPose, steps, potatoObject{i}, potatoVertices{i}, false);
% 
%     % Step 2: Close the gripper to grip the tomato
%     potatoPose = potatoGroundPos(i, :);
%     RobotClass.MoveObject2(harvesterBot, potatoPose, steps, potatoObject{i}, potatoVertices{i}, true);
% 
%     % Step 2: move tomato to the unsorted crate pose
%     unsortedCratePose = unsortedPos(i+3, :) + [0,0,0.1];
%     RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, potatoObject{i}, potatoVertices{i}, true);
% 
%     % Step 4: move EE pose to point down (z axis down) and lower tomato in
%     % crate
%     unsortedCratePose = unsortedPos(i+3, :) + [0,0,potatoSize(:,3)];
%     RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, potatoObject{i}, potatoVertices{i}, true);
% 
%     % Step 5: release gripper
%     unsortedCratePose = unsortedPos(i+3, :);
%     RobotClass.MoveObject2(harvesterBot, unsortedCratePose, steps, potatoObject{i}, potatoVertices{i}, false);    
% 
% end


%% Harvesting Tomatoes

% Update ellipsoids for the initial configuration
collisionHandler.updateEllips(harvesterBot);

for i = 1:size(tomatoTreePos, 1)
    % Step 1: Move to the tomato position, with room for the gripper.
    tomatoPose = tomatoTreePos(i, :) + [0, 0, 0.1];

    % Check for collision before moving to the tomato position
    collisionDetected = collisionHandler.collisionCheck(tomatoPose);
    if collisionDetected
        warning('Collision detected while approaching the tomato!');
        continue; % Skip to the next tomato if a collision is detected
    end

    % Move to the tomato position
    RobotClass.MoveRobot(harvesterBot, tomatoPose, steps, tomatoObject{i}, tomatoVertices{i}, false, 'forward', right, left);
    % Update ellipsoids after reaching the tomato position
    collisionHandler.updateEllips(harvesterBot);

    % Step 2: Move to the exact tomato position and pick it up
    RobotClass.MoveRobot(harvesterBot, tomatoTreePos(i, :), steps, tomatoObject{i}, tomatoVertices{i}, true, 'down', right, left);
    collisionHandler.updateEllips(harvesterBot); % Update ellipsoids after picking up

    % Move the tomato to the unsorted crate position
    unsortedCratePose = unsortedPos(i, :) + [0, 0, 0.1];
    if ~collisionHandler.collisionCheck(unsortedCratePose)
        RobotClass.MoveRobot(harvesterBot, unsortedCratePose, steps, tomatoObject{i}, tomatoVertices{i}, true, 'forward', right, left);
        collisionHandler.updateEllips(harvesterBot); % Update ellipsoids after moving to crate
    end

    % Clear ellipsoids when finished
    collisionHandler.clearEllipsoids();
end

