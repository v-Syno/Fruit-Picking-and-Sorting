function [EEDirection,pickupOffset,currentObject,...
    currentVertices,tomatoIndex,potatoIndex] = EEDirection(produceTags,i,gripperLength,gripperHeight,...
    tomatoObject,tomatoVertices,potatoObject,potatoVertices,tomatoIndex,potatoIndex)

    if strcmp(produceTags{i}, 'Tomatoes')
        EEDirection = pointForwards;
        pickupOffset = [0, -gripperLength, gripperHeight]; % Offset for tomatoes on the tree.
        currentObject = tomatoObject{tomatoIndex};
        currentVertices = tomatoVertices{tomatoIndex};
        tomatoIndex = tomatoIndex + 1;  % Increment tomato index only
    elseif strcmp(produceTags{i}, 'Potatoes')
        EEDirection = pointDown;
        pickupOffset = [0, 0, gripperLength]; % Offset for potatoes on the ground.
        currentObject = potatoObject{potatoIndex};
        currentVertices = potatoVertices{potatoIndex};
        potatoIndex = potatoIndex + 1;  % Increment potato index only
    end
end