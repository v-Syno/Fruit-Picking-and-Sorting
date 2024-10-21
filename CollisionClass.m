classdef CollisionClass
    methods (Static)

        % Using LPI, check if any of the links will collide with the ground
        function check = LPIGroundCheck(robot)            
            planeNormal = [0,0,1];
            pointOnPlane = [0,0,0];

            check = 0;

            if (strcmp(robot.plyFileNameStem, 'ColouredPanda'))
                leftGripper = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,-0.127,0.215);
                rightGripper = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,0.127,0.215);

            elseif (strcmp(robot.plyFileNameStem, 'LinearUR3e'))
                leftGripper = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,0.127,0.2312);
                rightGripper = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,-0.127,0.2312);
            else
                disp("Error, no robot found");
            end

            point1 = [leftGripper(1,4), leftGripper(2,4), leftGripper(3,4)];
            point2 = [rightGripper(1,4), rightGripper(2,4), rightGripper(3,4)];

            u = point2 - point1;
            w = point1 - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);

            if abs(D) < 10^-7           % The segment is parallel to plane
                if N == 0               % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;          % No intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;

            if (sI < 0 || sI > 1)
                check = 3;              %The intersection point  lies outside the segment, so there is no intersection
            else
                check = 1;
            end
        end
       
        
function [qMatrix, restartLoop] = AvoidCollision(robot, qMatrix, i, steps, q2)
    % Adjusts the robot's path using `jtraj` for smooth motion.

    disp('(Potential) Collision detected! Adjusting path...');

    % Get the current pose of the robot.
    poseNow = robot.model.getpos();
    pointA = robot.model.fkineUTS(qMatrix(max(i-1, 1), :));
    pointNext = robot.model.fkineUTS(qMatrix(i, :));
    pointAvec = pointA(1:3, 4);
    nextpointAvec = pointNext(1:3, 4);

    % Calculate direction vector and normalize.
    targetVec = nextpointAvec - pointAvec;
    magn = norm(targetVec);
    if magn < 1e-6
        targetVec = [0, 0, 1];
        magn = norm(targetVec);
    end
    normalisedTarg = targetVec / magn;

    % Create a new point away from the collision.
    targetDist = -0.1;
    newPoint = pointAvec + targetDist * normalisedTarg;
    newPoint(3) = newPoint(3) + 0.1;

    % Calculate the inverse kinematics for the adjusted target.
    adjustedTransform = transl(newPoint);
    poseAvoid = robot.model.ikcon(adjustedTransform, poseNow);

    % Ensure that poseNow and poseAvoid are proper joint angle vectors.
    poseNow = poseNow(:)';
    poseAvoid = poseAvoid(:)';

    % Generate trajectory using `jtraj` for smooth motion.
    sidesteps = 10;
    firstqMatrix = jtraj(poseNow, poseAvoid, sidesteps);
    secondqMatrix = jtraj(firstqMatrix(end, :), q2, steps - sidesteps);

    % Combine the two parts into the new `qMatrix`.
    qMatrix = [firstqMatrix; secondqMatrix];
    restartLoop = true;
end




    end
        
end
