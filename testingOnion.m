% Onion positions
onionTreePos = [
    0.5, 1.0, 0.8;
    -0.7, 0.9, 0.9;
    0.6, 1.2, 1.1;
    -0.8, 0.8, 1.0;    
    ];

% Load the onion model file
onionFile = 'Red_onion_with_colors.ply';

% Place the onions in the environment
for i = 1:size(onionTreePos, 1)
    [onionObject, onionVertices] = Fruit.PlaceObjects2(onionFile, onionTreePos(i, :), 'Scale', [1.5, 1.5, 1.5], 'Rotate', [0, 0, 0]);
end
