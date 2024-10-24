function [tomatoTreePos,unsortedBoxPos,boxRight,boxLeft] = PlantLocations(tableZ)

    % Tomatoes on trees
    tomatoTreePos = [
        0.55, 1.11, 0.35;
        0.5, 1.1, 0.4;
        0.4, 1.15, 0.26; 
    
        -0.4, 1.15, 0.16;  
    
        -0.6, 1.1, 0.35;
        -0.7, 1.05 0.4;  
        ];
    

    % % Potatoes
    % potatoGroundPos = [
    %     0.55, 1.11, 0.01;
    %     0.5, 1.27, 0.01;
    %     0.35, 1.2, 0.01;    
    %     ];
    
    % Unsorted box 
    boxY_1 = -0.08;  % row 1
    boxY_2 = 0.08;   % row 2
    boxZ = 0.06;

    unsortedBoxPos = [
        0.15, boxY_1, boxZ;
        0, boxY_1, boxZ;
        -0.15, boxY_1, boxZ;

        0.15, boxY_2, boxZ;
        0, boxY_2, boxZ;
        -0.15, boxY_2, boxZ;
        ];
    
    % Sorted box positions
    boxRight = [
        0.95,-0.8,tableZ,
        0.85,-0.8,tableZ,
        0.75,-0.8,tableZ,
        ];
    
    boxLeft = [
        0.55,-0.8,tableZ,
        0.45,-0.8,tableZ,
        0.35,-0.8,tableZ,
        ];

end