classdef GripLeft < RobotBaseClass
    %% Create Grip finger Robot for Left side of Robitiq 2F-140

    % Referenced from A1 grip and Robotics toolbox bots 

    properties(Access = public)              
        plyFileNameStem = 'GripLeft';
    end
    
    methods
%% Define robot Function 
function self = GripLeft(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();
            
        end

%% Create the robot model
        function CreateModel(self)   
            % Left Gip DH Parameters

            link(1) = Link('d',0,'a',0.1,'alpha',0,'offset',deg2rad(40.07),'qlim', [deg2rad(-40.07),0]);
            link(2) = Link('d',0,'a',0.0065,'alpha',0,'offset',deg2rad(-40.07),'qlim', [0,deg2rad(40.07)]);
            link(3) = Link('d',0,'a',0.0605,'alpha',0,'offset',0,'qlim', [0,0]);

            self.model = SerialLink(link,'name',self.name);

            q = zeros(1,3);
        

        end
    end
end