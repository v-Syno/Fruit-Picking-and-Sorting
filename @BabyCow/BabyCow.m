classdef BabyCow < RobotBaseClass
    properties (Access = public)
    plyFileNameStem = 'BabyCow';
    end
    
    methods
        %% Define robot Function 
        function self = BabyCow(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;            
            self.PlotAndColourRobot();         
        end
     %% Create the robot model
        function CreateModel(self)
            link(1) = Link('alpha',pi/2,'a',0,'d',0.3,'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end
    end    
end