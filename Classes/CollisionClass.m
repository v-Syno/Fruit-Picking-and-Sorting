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
        function self = CollisionClass()
            % Constructor for CollisionClass.
            % It can initialize any required parameters or properties.
        end

        function visualiseEllips(self, robot, dur)
            hold on
            q0 = robot.model.getpos;
            [~,jointTr]= robot.model.fkine(q0);
            
            jointLinks = robot.model.links;
            for i = 2:length(jointTr)
                centre = .5*(jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
                
            end
            if strcmp(robot.plyFileNameStem,'ColouredPanda')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.15 0.001 .110 0.071 .095 0.131 .095 0.091 0.051 .041 0.001];
                rote = jointTr(7).tr2rpy;
                jointAdj = transl(self.centreJoints{7});
                jointAdj = jointAdj*trotz(rote(3))*troty(rote(2))*trotx(rote(1))*transl(0,0.1,0);
                self.centreJoints{7} = jointAdj(1:3, 4);
            end
            if strcmp(robot.plyFileNameStem,'LinearUR3e')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.12 .158 .075 .031 .032 .0853 .031];
                rote = jointTr(2).tr2rpy;
                jointAdj = transl(self.centreJoints{2});
                jointAdj = jointAdj*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                self.centreJoints{2} = jointAdj(1:3, 4);
            end
            
            for j = 1:length(self.centreJoints)
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3), self.jointX, self.jointY, self.linkLeng(j)); % generate points per link
                self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ); % generates the ellipse
                Jrot = rad2deg(jointTr(j).tr2rpy);
                  if j == 2 && strcmp(robot.plyFileNameStem,'LinearUR3e')
                      Jrot(1) = Jrot(1)+90;
                  end
                rotate(self.JointEllipse(j), [0 0 1], Jrot(3), self.centreJoints{j});
                rotate(self.JointEllipse(j), [0 1 0], Jrot(2), self.centreJoints{j});
                rotate(self.JointEllipse(j), [1 0 0], Jrot(1), self.centreJoints{j});
               
            end
            pause(dur)
            delete(self.JointEllipse);
        end
        
        function updateEllips(self, robot)
            hold on
            q0 = robot.model.getpos;
            [~,jointTr] = robot.model.fkine(q0);
            
            jointLinks = robot.model.links;
            for i = 2:length(jointTr)
                centre = .5*(jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
                
            end
            if strcmp(robot.plyFileNameStem,'ColouredPanda')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.15 0.001 .110 0.071 .095 0.131 .095 0.091 0.051 .041 0.001];
                rote = jointTr(7).tr2rpy;
                jointAdj = transl(self.centreJoints{7});
                jointAdj = jointAdj*trotz(rote(3))*troty(rote(2))*trotx(rote(1))*transl(0,0.1,0);
                self.centreJoints{7} = jointAdj(1:3, 4);
            end
            if strcmp(robot.plyFileNameStem,'LinearUR3')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.12 .158 .0935 .031 .032 .0853 .031];
                rote = jointTr(2).tr2rpy;
                jointAdj = transl(self.centreJoints{2});
                jointAdj = jointAdj*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                self.centreJoints{2} = jointAdj(1:3, 4);
            end
            
            for j = 1:length(self.centreJoints)
                self.centreJoints{j};          
            end
        end

        function check = collisionGroundLPI(self, robot)
            % collisionGroundLPI checks if the robot's gripper intersects with the ground plane.
            planeNormal = [0, 0, 0.1];
            pointOnPlane = [0, 0, 0.01];  % Ground plane is at z = 0.01.

            T_EE = robot.model.fkineUTS(robot.model.getpos());
        
            % Define the line segment points based on the robot type.
            if strcmp(robot.plyFileNameStem, 'ColouredPanda')
                leftGripper = T_EE * transl(0,-0.127,0.215);
                rightGripper = T-EE * transl(0,0.127,0.215);

            elseif strcmp(robot.plyFileNameStem, 'LinearUR3e')
                leftGripper = T_EE * transl(0,0.127,0.2312);
                rightGripper = T_EE * transl(0,-0.127,0.2312);
            else
                error('Unsupported robot type.');
            end

            point1 = leftGripper(1:3,4)' ;
            point2 = rightGripper(1:3,4)' ;
        
            % Use the LinePlaneIntersection function to find the intersection.
            [~, check] = LinePlaneIntersection(planeNormal, pointOnPlane, point1, point2);
       
        end

    end
end
