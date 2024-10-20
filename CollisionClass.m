classdef CollisionClass
    methods (Static)

        %% Line Plane Intersection for Ground
        % Given a plane (normal and point) and two points that make up another line, get the intersection
        % Check == 0 if there is no intersection
        % Check == 1 if there is a line plane intersection between the two points
        % Check == 2 if the segment lies in the plane (always intersecting)
        % Check == 3 if there is intersection point which lies outside line segment
        function check = LPIGround(robot) % Modified LinePlaneIntersection from toolbox

            planeNormal = [0,0,1];
            pointOnPlane = [0,0,0.01];

            if (strcmp(robot.plyFileNameStem, 'ColouredPanda'))
                leftGripper = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,-0.127,0.215);
                rightGripper = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,0.127,0.215);

            elseif (strcmp(robot.plyFileNameStem, 'LinearUR3e'))
                leftGripper = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,0.127,0.2312);
                rightGripper = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,-0.127,0.2312);
            else
                error('Unkown robot type');
            end

            point1 = [leftGripper(1,4), leftGripper(2,4), leftGripper(3,4)];
            point2 = [rightGripper(1,4), rightGripper(2,4), rightGripper(3,4)];

            u = point2 - point1;
            w = point1 - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);

            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7           % The segment is parallel to plane
                if N == 0               % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;          % no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;

            if (sI < 0 || sI > 1)
                check = 3;               %The intersection point lies outside the segment, so there is no intersection
            else
                check = 1;
            end
        end

        function [qMatrix, restartLoop] = AvoidCollision(robot, qMatrix, i, steps, q2)
            % AvoidCollision Adjusts the robot's path if a collision is detected.
            %
            % Inputs:
            %   - robot: The robot model.
            %   - qMatrix: Current trajectory matrix.
            %   - i: Current step in the trajectory.
            %   - steps: Total steps for the movement.
            %   - q2: Target joint configuration.
            %
            % Outputs:
            %   - qMatrix: Updated trajectory matrix.
            %   - restartLoop: Boolean to indicate if the loop should restart.
        
            disp('(Potential) Collision detected! Adjusting path...');
        
            % Get current pose and the previous point
            poseNow = robot.model.getpos();
            pointA = robot.model.fkineUTS(qMatrix(max(i-1, 1), :));
            pointNext = robot.model.fkineUTS(qMatrix(i, :));
            
            % Define the direction vector from current point to next point
            pointAvec = pointA(1:3, 4); % Current position in Cartesian space
            nextpointAvec = pointNext(1:3, 4); % Next point in Cartesian space
            targetVec = nextpointAvec - pointAvec; % Direction vector
            normalisedTarg = targetVec / norm(targetVec); % Normalize the direction
        
            % Define a new point that moves away from the ground
            targetDist = -0.1; % Distance to move away from the collision (negative to back away)
            newPoint = pointAvec + targetDist * normalisedTarg;
        
            % Lift the new point slightly to avoid ground
            newPoint(3) = newPoint(3) + 0.1; % Lift the point in the z-axis to avoid collision
        
            % Create a transformation matrix for the new target point
            adjustedTransform = transl(newPoint);
        
            % Compute the inverse kinematics for the adjusted point
            poseAvoid = robot.model.ikcon(adjustedTransform, poseNow);
        
            % Create a smooth trajectory to avoid the collision point
            sidesteps = 10; % Number of steps for the avoidance part of the trajectory
            s1 = lspb(0, 1, sidesteps);
            firstqMatrix = (1 - s1) * poseNow + s1 * poseAvoid;
        
            % Create a trajectory to return to the original goal after avoiding the obstacle
            s2 = lspb(0, 1, steps - sidesteps);
            secondqMatrix = (1 - s2) * firstqMatrix(end, :) + s2 * q2;
        
            % Combine the two trajectories
            qMatrix = [firstqMatrix; secondqMatrix];
            
            % Set restartLoop to true to indicate that the loop should restart with the new path
            restartLoop = true;
        end


    end
end
