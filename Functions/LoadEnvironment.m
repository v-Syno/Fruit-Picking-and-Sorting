function LoadEnvironment

    %% Variables
    
    tableX = 0.5;
    tableY = -0.95;

    %% Essentials
    floor = imread ('grass.jpg'); % Floor Image
    soil = imread ('soil.jpg'); % Soil Image
    wall = imread ('sky.PNG'); % wall Image
    
    surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01] ,'CData',floor ,'FaceColor','texturemap'); %Floor
    surf([-1.8,-1.8;1.8,1.8],[1,1.8;1,1.8],[0.01,0.01;0.01,0.02] ,'CData',soil ,'FaceColor','texturemap') % Soil - veggie patch
    surf([-1.8, 1.8; -1.8, 1.8], [1.8, 1.8; 1.8, 1.8], [1.8, 1.8; 0, 0] ,'CData',wall ,'FaceColor','texturemap'); % Back wall (y = 1.8)
    
    %% Trees
    tree = 'treeSkinny.ply';
    ObjectClass.PlaceObjects2(tree, [-0.7,1.3,0.01], 'Scale', [0.15,0.15,0.25]); % left tree (big)
    ObjectClass.PlaceObjects2(tree, [-0.4,1.3,0.01], 'Scale', [0.1, 0.1,0.2]); % right tree (small)
    ObjectClass.PlaceObjects2(tree, [0.5,1.3,0.01], 'Scale', [0.15,0.15,0.25]); % right tree (big)

    %% Robot platforms
    slab = 'table.ply';
    ObjectClass.PlaceObjects2(slab, [0,0.6,-0.2], 'Scale', [0.6,0.4,0.5]);
    ObjectClass.PlaceObjects2(slab, [0.75,-0.25,-0.2], 'Scale', [0.2,0.2,0.5]);

    ObjectClass.PlaceObjects2(slab, [0,0,-0.2], 'Scale', [0.34,0.35,0.5]); % platform for the objects
    
    %% Crates
    crate = 'crate.ply';
    ObjectClass.PlaceObjects2(crate, [0,0,0.1], 'Scale', [0.5,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted
    ObjectClass.PlaceObjects2(crate, [0.85,-0.8,0.55], 'Scale', [0.5,0.5,0.5]); % right crate
    ObjectClass.PlaceObjects2(crate, [0.45,-0.8,0.55], 'Scale', [0.5,0.5,0.5]); % left crate
    
    %% Table
    table = 'table.ply';
    ObjectClass.PlaceObjects2(table, [tableX,tableY,0], 'Scale', [0.5,0.4,1]);
    
    %% Scarecrow
    scarecrow = 'scarecrow.ply';
    ObjectClass.PlaceObjects2(scarecrow, [0,1.3,0], 'Scale', [0.4,0.4,0.4]);
    
    %% Safety Features
    
    % Guard rails to secure the zone from external hazards
    barrier = 'barrier.ply';
    ObjectClass.PlaceObjects2(barrier, [1.2,-1.5,0], 'Scale', [1, 1, 0.5]);
    ObjectClass.PlaceObjects2(barrier, [-0.3,-1.5,0], 'Scale', [1, 1, 0.5]);
    ObjectClass.PlaceObjects2(barrier, [-1.8,-1.5,0], 'Scale', [1, 1, 0.5]);
    
    % warning signs
    sign1 = imread('sign1.png'); % robot sign
    sign2 = imread('sign2.png'); % PPE sign
    
    surf([0.7, 1.7; 0.7, 1.7], [-1.56, -1.56; -1.56, -1.56], [0.5, 0.5; 0.1, 0.1], 'CData', sign1, 'FaceColor', 'texturemap'); % Robot sign protection
    surf([-0.75, 0.25; -0.75, 0.25], [-1.51, -1.51; -1.51, -1.51], [0.5, 0.5; 0.1, 0.1], 'CData', sign2, 'FaceColor', 'texturemap'); % PPE sign protection
    
    % Estop
    estop = 'estop.ply';
    ObjectClass.PlaceObjects2(estop, [tableX-0.4,tableY-0.15,0.5], 'Scale', [0.5,0.5,0.5]);
    
    %% Light curtains

    % lightCurtain = 'lightCurtain_PosY.ply';
    % ObjectClass.PlaceObjects2(lightCurtain, [0,0,0], 'Scale', [1,1,1], 'Rotate',[0,0,pi/2]);
end