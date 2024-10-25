function SortTomatoes(sortingBot, unsortedBox, goodBox, badBox, tomatoObject, tomatoVertices, rightSorter, leftSorter)
    % Initialize the Robot Control.
    RobotControl = RobotClass();

    % Sorted box positions for EE.
    goodBoxEEPose = [0.6, -1.75, 0.6];
    badBoxEEPose = [0.4, -1.75, 0.6]; % Base position for bad box.

    % Gripper variables.
    gripperLength = 0.24;
    gripperHeight = 0.025;

    % Orientation for the end-effector.
    pointPosY = trotx(270, 'deg');
    pointDown = trotx(180, 'deg');
    pointNegY = trotx(90, 'deg');

    % Maximum capacity for each box.
    maxTomatoes = 3;
    goodTomatoCount = 0;
    badTomatoCount = 0;

    % Number of steps for smooth movement.
    steps = 75;

    % Loop through each item in the unsorted box.
    for i = 1:size(unsortedBox, 1)
        % Decide which box to use (good or bad).
        if goodTomatoCount < maxTomatoes && (badTomatoCount >= maxTomatoes || mod(i, 2) == 0)
            % If the good box has space, and either the bad box is full or we have an even count.
            targetBoxEEPose = goodBoxEEPose;
            targetBox = goodBox;
            targetCount = goodTomatoCount + 1;
            goodTomatoCount = goodTomatoCount + 1;
            disp('Sorting into good box.');
        else
            % Otherwise, place in the bad box with gripper offset for better reach.
            targetBoxEEPose = badBoxEEPose + [0, -gripperLength, 0]; % Apply gripper offset in Y direction.
            targetBox = badBox;
            targetCount = badTomatoCount + 1;
            badTomatoCount = badTomatoCount + 1;
            disp('Sorting into bad box.');
        end

        % Define positions for this loop.
        startPos = unsortedBox(i, :) + [0, 0, gripperLength + 0.1]; % Hover above the item.
        pickupItem = unsortedBox(i, :) + [0, 0, gripperLength]; % Position for picking up.
        liftPos = pickupItem + [0, 0, 0.8]; % Lift position after pickup.
        dropOffPos = targetBoxEEPose + [0, 0, 0.1]; % Hover above the target box.
        finalPos = targetBox(min(targetCount, size(targetBox, 1)), :) + [0, 0, gripperHeight]; % Lower to the height of the target box.

        % Step 1: Move to hover above the unsorted item.
        RobotControl.MoveRobot(sortingBot, startPos, steps, [], [], false, rightSorter, leftSorter, pointDown);

        % Step 2: Lower to the item position (simulate pickup).
        RobotControl.MoveRobot(sortingBot, pickupItem, steps, tomatoObject{i}, tomatoVertices{i}, false, rightSorter, leftSorter, pointDown);
        RobotControl.GripperMove(rightSorter, leftSorter, 'close'); % Simulate gripping.

        % Step 3: Move back up after gripping.
        RobotControl.MoveRobot(sortingBot, liftPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointDown);

        % Step 4: Move to hover above the target drop-off position.
        RobotControl.MoveRobot(sortingBot, dropOffPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointNegY);

        % Step 5: Lower to the drop-off position.
        RobotControl.MoveRobot(sortingBot, finalPos, steps, tomatoObject{i}, tomatoVertices{i}, true, rightSorter, leftSorter, pointNegY);

        % Step 6: Release the object.
        RobotControl.GripperMove(rightSorter, leftSorter, 'open');
        ObjectClass.DropObject(sortingBot, tomatoObject{i}, tomatoVertices{i}, finalPos);

        % Step 7: Move back up to a safe hover position after releasing the object.
        RobotControl.MoveRobot(sortingBot, dropOffPos + [0, 0, 0.5], steps, [], [], false, rightSorter, leftSorter, pointDown);

        % Stop if both boxes are full.
        if goodTomatoCount >= maxTomatoes && badTomatoCount >= maxTomatoes
            disp('Both boxes are full. Stopping sorting process.');
            break;
        end
    end
end
