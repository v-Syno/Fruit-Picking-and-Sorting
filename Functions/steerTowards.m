function qNew = steerTowards(qNear, qRand, stepSize)
    % Moves from qNear towards qRand by a distance stepSize
    direction = qRand - qNear;
    direction = direction / norm(direction); % Normalize direction vector
    qNew = qNear + stepSize * direction; % Move a step towards qRand
end
