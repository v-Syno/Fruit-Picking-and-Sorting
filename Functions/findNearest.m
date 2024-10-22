function [qNear, nearIdx] = findNearest(tree, qRand)
    % Find the nearest node in the tree to qRand
    distances = vecnorm(tree - qRand, 2, 2); % Euclidean distances to qRand
    [~, nearIdx] = min(distances); % Index of the nearest point
    qNear = tree(nearIdx, :);
end
