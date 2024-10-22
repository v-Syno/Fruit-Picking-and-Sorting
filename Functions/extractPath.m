function qPath = extractPath(tree, parent)
    % Extracts the path from qStart to qGoal using the parent array
    qPath = [];
    currentIdx = size(tree, 1); % Start at the last node (goal)

    while currentIdx > 0
        qPath = [tree(currentIdx, :); qPath]; % Add node to the path
        currentIdx = parent(currentIdx); % Move to parent node
    end
end
