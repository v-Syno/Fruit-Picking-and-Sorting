classdef Panda < RobotBaseClass
    properties(Access = public)    
        plyFileNameStem = 'ColouredPanda';
    end
    
    methods 
%% Define robot Function 
        function self = Panda(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;            
            self.PlotAndColourRobot();         
        end
%% Create the robot model
        function CreateModel(self)

%         % Create Panda model
            link(1) = Link('d',0.333,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-pi, pi]); %GOOD 1
            link(2) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim', [-2.26893, 2.07694]); %GOOD 2
            link(3) = Link('d',0.316,'a',0,'alpha',pi/2,'offset',0,'qlim', [-pi, pi]); %GOOD 3
            link(4) = Link('d',0,'a',0.0825,'alpha',0,'offset',0,'qlim', [0, 0]); %GOOD % dummy offset link
            link(5) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2.1293, 2.0944]); %GOOD 4
            link(6) = Link('d',0,'a',0.0825,'alpha',-pi/2,'offset',0,'qlim', [0, 0]); %GOOD %dummy offset link
            link(7) = Link('d',0.125,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]); %GOOD % 5
            link(8) = Link('d',0.2613,'a',0,'alpha',pi/2,'offset',0,'qlim', [0, 0]);%GOOD %dummy offset link
            link(9) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]);%GOOD 6
            link(10) = Link('d',0,'a',-0.09,'alpha',-pi/2,'offset',-pi/2,'qlim', [0, 0]);% %dummy offset link
            link(11) = Link('d',0.105,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]);%GOOD 7        


            self.model = SerialLink(link,'name',self.name);
        end      
    end
end