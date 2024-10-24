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
    unsortedBoxPos = [
        0.15, 0.025, 0.01;
        0, 0.025, 0.01;
        -0.15, 0.025, 0.01;
        0.15, 0.18, 0.01;
        0, 0.18, 0.01;
        -0.15, 0.18, 0.01;
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