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

        function [producePositions,tomatoObject,tomatoVertices,produceTags] = GenerateTomatoes()
            global producePositions produceTags;
            tomato = 'tomato.ply';
            Object = ObjectClass();

            % Reset arrays for fresh generation
            producePositions = [];
            produceTags = {};

            ProduceClass.ClearProduce();
        
            % Tomatoes on trees
            tomatoTreePos = [
                -0.7, 1.4, 0.35;
                0, 1.83,0.4;
                % 0.3,1.55,0.45
                0.63,1.43,0.5;
                ];

            [tomatoObject, tomatoVertices] = Object.PlaceObjects(tomato, tomatoTreePos);

            % Tag each potato object for easy identification and future deletion.
            for i = 1:length(tomatoObject)
                set(tomatoObject{i}, 'Tag', 'Tomatoes');
            end

            producePositions = [producePositions; tomatoTreePos]; % Append positions
            produceTags = [produceTags; repmat({'Tomatoes'}, size(tomatoTreePos, 1), 1)]; % Append 'Tomatoes' tags

        end

        function [producePositions, potatoObject, potatoVertices,produceTags] = GeneratePotatoes()
            global producePositions produceTags;
            potato = 'potato.ply';
            Object = ObjectClass();

            % Reset arrays for fresh generation
            producePositions = [];
            produceTags = {};
        
            ProduceClass.ClearProduce();

            % Define positions for the new potatoes.
            potatoGroundPos = [
                -0.7, 0.75, 0.01;
                -0.3, 1.5, 0.01;
                0.5,1.25,0.01
            ];
        
            % Generate new potato objects at the specified positions.
            [potatoObject, potatoVertices] = Object.PlaceObjects(potato, potatoGroundPos);
        
            % Tag each potato object for easy identification and future deletion.
            for i = 1:length(potatoObject)
                set(potatoObject{i}, 'Tag', 'Potatoes');
            end
            producePositions = [producePositions; potatoGroundPos]; % Append positions
            produceTags = [produceTags; repmat({'Potatoes'}, size(potatoGroundPos, 1), 1)]; % Append 'Potatoes' tags
        end

        function [producePositions,potatoObject,potatoVertices,tomatoObject,tomatoVertices,produceTags] = GenerateMix()
            global producePositions produceTags;
            potato = 'potato.ply';
            tomato = 'tomato.ply';

            % Reset arrays for fresh generation
            producePositions = [];
            produceTags = {};

            Object = ObjectClass();

            ProduceClass.ClearProduce();

            % Tomatoes on trees
            mixedPos = [
                -0.7, 1.4, 0.35;    % tomato
                 0, 1.83, 0.4;      % tomato
                0.7, 1.4 ,0.5;      % tomato
                -0.7, 1.4, 0.01;    % potato
                -0.25, 1.4, 0.01;   % potato
                ];

            [tomatoObject, tomatoVertices] = Object.PlaceObjects(tomato, mixedPos(1:3,:));
            [potatoObject, potatoVertices] = Object.PlaceObjects(potato, mixedPos(4:5,:));

            % Tag each potato object for easy identification and future deletion.
            for i = 1:length(tomatoObject)
                set(tomatoObject{i}, 'Tag', 'Tomatoes');
            end

            % Tag each potato object for easy identification and future deletion.
            for i = 1:length(potatoObject)
                set(potatoObject{i}, 'Tag', 'Potatoes');
            end

            mixedTags = {'Tomatoes'; 'Tomatoes'; 'Tomatoes'; 'Potatoes'; 'Potatoes'}; % Matching tags
    
            producePositions = [producePositions; mixedPos]; % Append positions
            produceTags = [produceTags; mixedTags]; % Append mixed tags
        end

        function [unsortedBoxMix,unsortedBox,goodBox,badBox] = BoxLocations()
            % Positions for produce to 'fall' into when dropped into crate
            unsortedBoxMix = ProduceClass.GeneratePyramid(0,-0.2,0.43);

            unsortedBox = [
                -0.038, -0.24, 0.43;
                -0.038, -0.165, 0.43;
                0.038, -0.24, 0.43;
                ];

            
            goodBox = [
                0.7, -1.75, 0.45;
                0.6, -1.75, 0.45;
                0.5, -1.75, 0.45;
                ];
        
            badBox = [
                1.1, -1.15, 0.06;
                1.1, -1.2, 0.06;
                1.1, -1.25, 0.06;
                ];
        end
        
        function qualityLabels = RandomizeQuality(numProduce)
            % RandomizeQuality randomly assigns 'good' or 'bad' to a given number of objects.
            % numTomatoes: Total number of produce to assess.
            % qualityLabels: A cell array containing 'good' or 'bad' for each produce.
        
            % Initialize the output as a cell array.
            qualityLabels = cell(1, numProduce);
        
            % Iterate through each tomato.
            for i = 1:numProduce
                % Randomly assign 'good' or 'bad' to each tomato with a 50% chance.
                if rand() > 0.5
                    qualityLabels{i} = 'good';
                else
                    qualityLabels{i} = 'bad';
                end
            end
        end

        function ClearProduce()
            delete(findobj('Tag', 'Tomatoes'));
            delete(findobj('Tag', 'Potatoes'));
        end

    end
end