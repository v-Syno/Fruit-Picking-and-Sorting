classdef Panda < RobotBaseClass
    properties(Access = public)    
        plyFileNameStem = 'Panda';
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
            % Create Panda model
            link(1) = Link('d',0.333,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-pi, pi]);         % Main Joint 1  (1)
            link(2) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim', [-2.26893, 2.07694]);    % Main joint 2  (2)
            link(3) = Link('d',0.316,'a',0,'alpha',pi/2,'offset',0,'qlim', [-pi, pi]);          % Main Joint 3  (3)
            link(4) = Link('d',0,'a',0.0825,'alpha',0,'offset',0,'qlim', [0, 0]);               % Dummy offset link
            link(5) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2.1293, 2.0944]);         % Main Joint 4  (5)
            link(6) = Link('d',0,'a',0.0825,'alpha',-pi/2,'offset',0,'qlim', [0, 0]);           % Dummy offset link 2
            link(7) = Link('d',0.125,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]);             % Main Joint 5  (7)
            link(8) = Link('d',0.2613,'a',0,'alpha',pi/2,'offset',0,'qlim', [0, 0]);            % Dummy offset link 3
            link(9) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]);                 % Main Joint 6  (9)
            link(10) = Link('d',0,'a',-0.09,'alpha',-pi/2,'offset',-pi/2,'qlim', [0, 0]);       % Dummy offset link 4
            link(11) = Link('d',0.105,'a',0,'alpha',0,'offset',0,'qlim', [-pi, pi]);            % Main Joint 7  (11)


            self.model = SerialLink(link,'name',self.name);
        end      
    end
end

% Values from Franka Emita Panda's github
% Dummy joints are needed as these current values dont build the robot correctly

% link(1) = Link('a',     0.0, 'd', 0.333, 'alpha',   0.0, 'qlim', [-2.8973 2.8973], ...
%             'm', 4.970684, 'r', [3.875e-03 2.081e-03 0], 'I', [7.03370e-01  7.06610e-01  9.11700e-03 -1.39000e-04  1.91690e-02  6.77200e-03], 'G', 1);
% link(2) = Link('a',     0.0, 'd',   0.0, 'alpha', -pi/2, 'qlim', [-1.7628 1.7628], ...
%             'm', 0.646926, 'r', [-3.141e-03 -2.872e-02 3.495e-03], 'I', [7.96200e-03  2.81100e-02  2.59950e-02 -3.92500e-03  7.04000e-04  1.02540e-02], 'G', 1);
% link(3) = Link('a',     0.0, 'd', 0.316, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
%             'm', 3.228604, 'r', [ 2.7518e-02 3.9252e-02 -6.6502e-02], 'I', [3.72420e-02  3.61550e-02  1.08300e-02 -4.76100e-03 -1.28050e-02 -1.13960e-02], 'G', 1);
% link(4) = Link('a',  0.0825, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-3.0718 -0.0698], ...
%             'm', 3.587895, 'r', [-5.317e-02 1.04419e-01 2.7454e-02], 'I', [2.58530e-02  1.95520e-02  2.83230e-02  7.79600e-03  8.64100e-03 -1.33200e-03], 'G', 1);
% link(5) = Link('a', -0.0825, 'd', 0.384, 'alpha', -pi/2, 'qlim', [-2.8973 2.8973], ...
%             'm', 1.225946, 'r', [-1.1953e-02 4.1065e-02 -3.8437e-02], 'I', [3.55490e-02  2.94740e-02  8.62700e-03 -2.11700e-03  2.29000e-04 -4.03700e-03], 'G', 1);
% link(6) = Link('a',     0.0, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-0.0175 3.7525], ...
%             'm', 1.666555, 'r', [6.0149e-02 -1.4117e-02 -1.0517e-02], 'I', [1.96400e-03  4.35400e-03  5.43300e-03  1.09000e-04  3.41000e-04 -1.15800e-03], 'G', 1);
% link(7) = Link('a',   0.088, 'd',   0.107, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], ...
%             'm', 7.35522e-01, 'r', [1.0517e-02 -4.252e-03 6.1597e-02 ], 'I', [1.25160e-02  1.00270e-02  4.81500e-03 -4.28000e-04 -7.41000e-04 -1.19600e-03], 'G', 1);