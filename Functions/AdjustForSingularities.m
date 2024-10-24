function qAdjusted = AdjustForSingularities(qCurrent, minBendAngle)
    qAdjusted = qCurrent;
    
    % Adjust the 3rd joint to ensure the elbow maintains a minimum bend.
    % This is common for robots like UR3e where the third joint represents the elbow.
    if abs(qCurrent(3)) < deg2rad(minBendAngle)
        qAdjusted(3) = sign(qCurrent(3)) * deg2rad(minBendAngle);
    end
end
