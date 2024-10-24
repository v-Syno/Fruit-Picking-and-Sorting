function LoadEnvironment
   
    %% Essentials
    floor = imread ('grass.jpg'); % Floor Image
    soil = imread ('soil.jpg'); % Soil Image
    wall = imread ('sky.PNG'); % wall Image
        
    surf([-2.5, -2.5; 2.5, 2.5], [-2.5, 2.5; -2.5, 2.5], [0.01, 0.01; 0.01, 0.01], 'CData', floor, 'FaceColor', 'texturemap'); % Floor
    surf([-2, -2; 2, 2], [0.05, 2.5; 0.05, 2.5], [0.011, 0.011; 0.011, 0.011], 'CData', soil, 'FaceColor', 'texturemap'); % Soil - veggie patch
    surf([-2.5, 2.5; -2.5, 2.5], [2.5, 2.5; 2.5, 2.5], [2.5, 2.5; 0, 0], 'CData', wall, 'FaceColor', 'texturemap'); % Back wall (y = 2.5)

    %% Trees
    tree = 'treeSkinny.ply';
    ObjectClass.PlaceObjects2(tree, [-0.7,1.3,0.01], 'Scale', [0.15,0.15,0.25]); % left tree (big)
    ObjectClass.PlaceObjects2(tree, [-0.4,1.3,0.01], 'Scale', [0.1, 0.1,0.2]); % right tree (small)
    ObjectClass.PlaceObjects2(tree, [0.5,1.3,0.01], 'Scale', [0.15,0.15,0.25]); % right tree (big)

    %% Robot platforms
    slab = 'table.ply';
    ObjectClass.PlaceObjects2(slab, [0,0.9,-0.2], 'Scale', [0.2,0.3,0.5]); % Panda

    %% Crates
    crate = 'crate.ply';
    ObjectClass.PlaceObjects2(crate, [0,-0.2,0.45], 'Scale', [0.6,1,0.5], 'Rotate', [0, 0, pi/2]); % unsorted

    ObjectClass.PlaceObjects2(crate, [0.5,-1.75,0.45], 'Scale', [0.6,0.5,0.5]); % good crate
    ObjectClass.PlaceObjects2(crate, [1,-1.25,0.06], 'Scale', [0.5,0.5,0.5],'Rotate',[0,0,pi/2]); % bin
    
    %% Table
    table = 'table.ply';
    ObjectClass.PlaceObjects2(table, [0.25,-0.25,0], 'Scale', [0.6,0.4,0.8]); % Table for unsorted and Linear UR3e
    ObjectClass.PlaceObjects2(table, [0.6,-0.8,0], 'Scale', [0.5,0.4,0.8], 'Rotate',[0,0,pi/2]); % table for unsorted and ur3e

    ObjectClass.PlaceObjects2(table, [0.5,-1.9,0], 'Scale', [0.4,0.4,0.8]); % good
    
    %% Scarecrow
    scarecrow = 'scarecrow.ply';
    ObjectClass.PlaceObjects2(scarecrow, [-1.5,2.1,0], 'Scale', [0.4,0.4,0.4], 'Rotate',[0,0,pi/12]);
    
    %% Safety Features
    
    % Guard rails to secure the zone from external hazards
    barrier = 'fence.ply';
    ObjectClass.PlaceObjects2(barrier, [-2.25,1.75,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,-pi/2]);
    ObjectClass.PlaceObjects2(barrier, [-2.25,0.25,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,-pi/2]);
    ObjectClass.PlaceObjects2(barrier, [-2.25,-1,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,-pi/2]);
    ObjectClass.PlaceObjects2(barrier, [-0.7,-1,0], 'Scale', [0.3, 1, 0.3]);
    ObjectClass.PlaceObjects2(barrier, [-0,-1,0], 'Scale', [0.3, 1, 0.3]);
    ObjectClass.PlaceObjects2(barrier, [-0,-1,0], 'Scale', [0.1, 1, 0.3],'Rotate',[0,0,pi/2]);
    ObjectClass.PlaceObjects2(barrier, [1.5,-1.5,0], 'Scale', [0.3, 1, 0.3]);
    ObjectClass.PlaceObjects2(barrier, [2.25,-1.5,0], 'Scale', [0.3, 1, 0.3]);

    ObjectClass.PlaceObjects2(barrier, [2.25,2.5,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,pi/2]);
    ObjectClass.PlaceObjects2(barrier, [2.25,1,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,pi/2]);
    ObjectClass.PlaceObjects2(barrier, [2.25,-0.05,0], 'Scale', [0.3, 1, 0.3],'Rotate',[0,0,pi/2]);

    hoe = 'hoe.ply';
    ObjectClass.PlaceObjects2(hoe, [2.3,-1.53,0], 'Scale', [0.5, 1, 0.5]);
    
    % warning signs
    sign1 = imread('sign1.png'); % robot sign
    sign2 = imread('sign2.png'); % PPE sign
    
    surf([-2.2, -1.2; -2.2, -1.2], [-1.03, -1.03; -1.03, -1.03], [0.4, 0.4; 0.05, 0.05], 'CData', sign1, 'FaceColor', 'texturemap'); % Robot sign protection
    surf([-1.15, -0.15; -1.15, -0.15], [-1.03, -1.03; -1.03, -1.03], [0.4, 0.4; 0.05, 0.05], 'CData', sign2, 'FaceColor', 'texturemap'); % PPE sign protection
    
    % Estop
    estop = 'estop.ply';
    ObjectClass.PlaceObjects2(estop, [0.2,-2,0.4], 'Scale', [0.5,0.5,0.5]);
    
    %% Light curtains

    % lightCurtain = 'lightCurtain_PosY.ply';
    % ObjectClass.PlaceObjects2(lightCurtain, [0,0,0], 'Scale', [1,1,1], 'Rotate',[0,0,pi/2]);
end