classdef ProduceClass
    %ProduceClass holds all functions related to produce
    %   Detailed explanation goes here

    methods (Static)

        % Generate a 2x2 pyramid for the produce to simulate stacking of produce
        function pyramid = GeneratePyramid(centerX, centerY, height)
            % Define the parameters
            produceDiameter = 0.07; % Diameter of the produce
            spacing = produceDiameter * 1.1; % Slightly more than diameter for spacing

        
            % Create the 2x2 base layout of tomatoes
            positions = [
                centerX - spacing/2, centerY - spacing/2, height; % Bottom left
                centerX - spacing/2, centerY + spacing/2, height; % Top left
                centerX + spacing/2, centerY - spacing/2, height; % Bottom right
                centerX + spacing/2, centerY + spacing/2, height; % Top right
            ];
        
            % Add the fifth produce on top of the 2x2 layout to form a pyramid
            topPosition = [centerX, centerY, height + produceDiameter]; % Height is equal to base height + diameter
        
            % Combine the positions
            pyramid = [
                positions; % Base layer (2x2)
                topPosition % Top layer (pyramid)
            ];
        end

        function [tomatoTreePos,tomatoObject,tomatoVertices] = GenerateTomatoes()
            tomato = 'tomato.ply';

            Object = ObjectClass();
        
            % Tomatoes on trees
            tomatoTreePos = [
                -0.7, 1.4, 0.4;
                0, 1.83,0.4;
                0.7,1.4,0.5;
                ];

        [tomatoObject, tomatoVertices] = Object.PlaceObjects(tomato, tomatoTreePos);

        end

        function [potatoGroundPos,potatoObject,potatoVertices] = GeneratePotatoes()
            potato = 'potato.ply';

            Object = ObjectClass();
        
            % Tomatoes on trees
            potatoGroundPos = [
                -0.7, 1.4, 0.01;
                0, 1.83,0.01;
                0.7,1.4,0.01;
                ];

        [potatoObject, potatoVertices] = Object.PlaceObjects(potato, potatoGroundPos);

        end
        
        function [unsortedBox,goodBox,badBox] = BoxLocations()
            % Positions for produce to 'fall' into when dropped into crate
            unsortedBox = ProduceClass.GeneratePyramid(0,-0.2,0.4);
            
            goodBox = [
                0.7, -1.75, 0.45;
                0.6, -1.75, 0.45;
                0.5, -1.75, 0.45;
                ];
        
            badBox = [
                0, 0, 0.06;
                0, 0, 0.06;
                0, 0, 0.06;
                ];
        end
    
    end
end