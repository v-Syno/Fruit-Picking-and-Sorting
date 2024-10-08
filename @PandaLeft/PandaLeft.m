classdef PandaLeft < RobotBaseClass
    % This class is used to display each finger of the gripper
  
    properties
        plyFileNameStem = 'PandaLeft';
    end
    
    methods
       %% Define robot Function 
        function self = PandaLeft(baseTr)
           if nargin < 1
              baseTr = eye(4);	
           end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            %self.model.base = self.model.base.T * baseTr; 
            self.PlotAndColourRobot();

        end

        %% Set up the model of the Gripper
        function CreateModel(self)
           link(1) = Link([0 0 0.005 0 0]);
           link(1).qlim = [-20 30]*pi/180;
           self.model = SerialLink(link,'name',self.name);
        end   
    end
end