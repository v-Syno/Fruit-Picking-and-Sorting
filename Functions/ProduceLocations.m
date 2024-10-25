function [tomatoTreePos, unsortedBox,goodBox,badBox] = ProduceLocations()

    % Tomatoes on trees
    tomatoTreePos = [
        -0.7, 1.4, 0.4;
        0, 1.83,0.4;
        0.7,1.4,0.5;
        ];
    
    % % Potatoes
    % potatoGroundPos = [
    %     0.55, 1.11, 0.01;
    %     0.5, 1.27, 0.01;
    %     0.35, 1.2, 0.01;    
    %     ];

    % Positions for produce to 'fall' into when dropped into crate

    unsortedBox = ProduceClass.GeneratePyramid(0,-0.2,0.4);

    % Positions for produce to 'fall' into when dropped into crate
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