classdef ProduceClass
    %ProduceClass holds all functions related to produce
    %   Detailed explanation goes here

    methods (Static)

        % Generate a 2x2x2 pyramid for the produce to simulate stacking of produce
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
                -0.7, 1.4, 0.35;
                -0.65, 1.45, 0.5;
                0, 1.83,0.4;
                0.3,1.55,0.45
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
                -0.25, 1.4, 0.01;
                0, 1.83,0.01;
                0.3,1.55,0.01
                0.7,1.4,0.01;
                ];

        [potatoObject, potatoVertices] = Object.PlaceObjects(potato, potatoGroundPos);

        end
        
        function [potatoGroundPos,potatoObject,potatoVertices,tomatoTreePos,tomatoObject,tomatoVertices] = GenerateMix()
            potato = 'potato.ply';
            tomato = 'tomato.ply';

            Object = ObjectClass();

            % Tomatoes on trees
            tomatoTreePos = [
                -0.7, 1.4, 0.35;
                0, 1.83,0.4;
                0.7,1.4,0.5;
                ];
        
            % Tomatoes on trees
            potatoGroundPos = [
                -0.7, 1.4, 0.01;
                -0.25, 1.4, 0.01;
                ];

        [tomatoObject, tomatoVertices] = Object.PlaceObjects(tomato, tomatoTreePos);
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
                1.25, -1.15, 0.06;
                1.25, -1.2, 0.06;
                1.25, -1.25, 0.06;
                ];
        end
        
        function boxLabel = AssignBox(goodBoxCount, badBoxCount)
            % AssignBox randomly assigns an item to the good or bad box.
            % goodBoxCount: Current number of items in the good box.
            % badBoxCount: Current number of items in the bad box.
            % boxLabel: Returns 'good' or 'bad' based on random assignment.
        
            % Define the maximum capacity of each box.
            maxItems = 3;
        
            % Check if either box is full.
            if goodBoxCount >= maxItems
                boxLabel = 'bad'; % If the good box is full, assign to the bad box.
            elseif badBoxCount >= maxItems
                boxLabel = 'good'; % If the bad box is full, assign to the good box.
            else
                % Randomly assign to either 'good' or 'bad' if both boxes have space.
                if rand() > 0.5
                    boxLabel = 'good';
                else
                    boxLabel = 'bad';
                end
            end
        end

    end
end