function qRand = randConfig(robot)
    % Generates a random joint configuration for the given robot
    qLimits = robot.model.qlim;
    qRand = qLimits(:, 1) + (qLimits(:, 2) - qLimits(:, 1)) .* rand(size(qLimits, 1), 1)';
end
