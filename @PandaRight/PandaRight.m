classdef PandaRight < RobotBaseClass
    % This class is used to display each finger of the gripper
  
    properties
        plyFileNameStem = 'PandaRight';
    end
    
    methods
       %% Define robot Function 
        function self = PandaRight(baseTr)
           if nargin < 1
              baseTr = eye(4);	
           end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
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