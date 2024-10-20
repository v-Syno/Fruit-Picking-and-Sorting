%% Collision functions

classdef CollisionClass
    properties (Access = protected)
        ellipX; 
        ellipY;
        ellipZ;
        jointX;
        jointY;
        linkLeng;
        JointEllipse;
        centreJoints;
    end

    methods
        function visualiseEllipse(self, robot, duration)
            % Function to visualize ellipses around the robot's links for collision checking
            hold on;
            tempAng = robot.model.getpos();
            [Tr, jointTr] = robot.model.fkine(tempAng);
            jointLinks = robot.model.links;

            % Calculate the centers of each joint/link and create ellipses
            for i = 2:length(jointTr)
                centre = 0.5 * (jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
            end

            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                self.jointX = 0.058;
                self.jointY = 0.058;
                self.linkLeng = [.15 0.001 .110 0.071 .095 0.131 .095 0.091 0.051 .041 0.001];
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3')
                self.jointX = 0.058;
                self.jointY = 0.058;
                self.linkLeng = [.12 .158 .075 .031 .032 .0853 .031];
            end

            % Draw ellipses around each link
            for j = 1:length(self.centreJoints)
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3), self.jointX, self.jointY, self.linkLeng(j));
                self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ);
                Jrot = rad2deg(jointTr(j).tr2rpy);

                % Rotate ellipses according to the joint angles
                rotate(self.JointEllipse(j), [0 0 1], Jrot(3), self.centreJoints{j});
                rotate(self.JointEllipse(j), [0 1 0], Jrot(2), self.centreJoints{j});
                rotate(self.JointEllipse(j), [1 0 0], Jrot(1), self.centreJoints{j});
            end

            pause(duration);
            delete(self.JointEllipse);
        end

        function outp = collisionCheckSelf(self, robot, Q)
            % Check if the end effector position (Q) is in collision with any of the robot's links
            tempAng = robot.model.getpos();
            [Tr, jointTr] = robot.model.fkine(tempAng);

            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                self.jointX = 0.088;
                self.jointY = 0.088;
                self.linkLeng = [.25 0.001 .160 0.071 .075 0.091 .095 0.091 0.051 .081 0.001];
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3')
                self.jointX = 0.062;
                self.jointY = 0.062;
                self.linkLeng = [.12 .158 .075 .111 .042 .0853 .031];
            end

            % Calculate grip positions
            point = robot.model.fkineUTS(Q) * transl(0, 0, 0.15);
            point1 = point * transl(0, 0.0127, 0.2312);
            point2 = point * transl(0, -0.0127, 0.2312);
            pont = robot.model.fkineUTS(Q) * transl(0, 0, 0.05);
            pont = pont(1:3, 4);
            point1 = point1(1:3, 4);
            point2 = point2(1:3, 4);

            grips = [linspace(pont(1), point1(1), 4)', linspace(pont(2), point1(2), 4)', linspace(pont(3), point1(3), 4)';
                     linspace(pont(1), point2(1), 4)', linspace(pont(2), point2(2), 4)', linspace(pont(3), point2(3), 4)'];

            % Check for collision between the end effector and each link
            for i = 1:length(self.centreJoints) - 1
                for j = 1:length(grips)
                    point = [grips(j, 1), grips(j, 2), grips(j, 3)]';
                    result = (point - self.centreJoints{i})' * ([self.jointX^-2, self.jointY^-2, self.linkLeng(i)^-2] .* eye(3)) * (point - self.centreJoints{i});

                    if result <= 1
                        outp = true; % Return true if collision detected
                        return;
                    end
                end
            end

            outp = false; % No collision
        end

        function outp = collisionCheckGrip(self, robot, P)
            % Check if the gripper collides with a specific point P
            endAn = robot.model.getpos();

            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                gripXY = 4 * 0.0127;
                gripZ = 0.05;
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3')
                gripXY = 4 * 0.0127;
                gripZ = 0.0612;
            end

            endTr = SE3(robot.model.fkine(endAn).T * transl(0, 0, gripZ + 0.18)); % End effector transform

            % Check if the point P is inside the gripper's bounding ellipsoid
            place = (P - endTr.t');
            result = place * ([gripXY^-2, gripXY^-2, (5 * gripZ)^-2] .* eye(3)) * place';

            outp = result <= 1; % Return true if collision detected
        end

        function outp = collisionCheckCow(self, robot, cow)
            % Check if the gripper collides with a cow model
            endAn = robot.model.getpos();
            cowPos = cow.model.base.T;
            cowPos(3, 4) = cowPos(3, 4) + 0.3;
            P = cowPos(1:3, 4);

            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                gripXY = 5 * 0.0127;
                gripZ = 0.05;
            elseif strcmp(robot.plyFileNameStem, 'LinearUR3')
                gripXY = 9 * 0.0127;
                gripZ = 0.0612;
            end

            endTr = SE3(robot.model.fkine(endAn).T * transl(0, 0, gripZ + 0.15)); % End effector transform
            place = (endTr.t' - P);
            result = place * ([0.34^-2, 0.2^-2, 0.22^-2] .* eye(3)) * place';

            outp = result <= 1; % Return true if collision detected
        end
    
        function check = collisionGroundLPI(self, robot)
            planeNormal = [0,0,1];
            pointOnPlane = [0,0,0];
            if (strcmp(robot.plyFileNameStem, 'ColouredPanda'))
                QAgrip1 = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,-0.127,0.215);
                QAgrip2 = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,0.127,0.215);
                point1 = [QAgrip1(1,4), QAgrip1(2,4), QAgrip1(3,4)];
                point2 = [QAgrip2(1,4), QAgrip2(2,4), QAgrip2(3,4)];
            end
            if (strcmp(robot.plyFileNameStem, 'LinearUR3'))
                harvestGrip1 = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,0.127,0.2312);
                harvestGrip2 = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,-0.127,0.2312);
                point1 = [harvestGrip1(1,4), harvestGrip1(2,4), harvestGrip1(3,4)];
                point2 = [harvestGrip2(1,4), harvestGrip2(2,4), harvestGrip2(3,4)];
            end
            u = point2 - point1;
            w = point1 - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);

            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
%             intersectionPoint = point1OnLine + sI.*u; % not needed

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
    
        function qMat = remakeTraj(self, robot, sideSteps, steps, q2)
            % using basic RRT
            poseA = robot.model.getpos();
            
            qRand = poseA + .11*(2 * rand(1,length(robot.model.links)) - 1) * pi/8;
           
            if strcmp(robot.plyFileNameStem, 'LinearUR3')
                qRand(1) = poseA(1);
            end 
            planner = self.collisionCheckSelf(robot, qRand);
            trials = 0;
            while planner == true
                qRand = poseA + .11*(2 * rand(1,length(robot.model.links)) - 1) * pi/8;
                if strcmp(robot.plyFileNameStem, 'LinearUR3')
                    qRand(1) = poseA(1);
                end
                planner = self.collisionCheckSelf(robot, qRand);
                trials = trials +1
            end
            avoidPoseA = qRand;
            s1 = lspb(0,1,sideSteps);
            firstqMatrix = (1-s1)*poseA + s1*avoidPoseA;
            s2 = lspb(0,1, steps - sideSteps);
            secondqMatrix = (1-s2)*firstqMatrix(sideSteps, :) + s2*q2;
            qMat = [firstqMatrix; secondqMatrix];

        end
    end
end

