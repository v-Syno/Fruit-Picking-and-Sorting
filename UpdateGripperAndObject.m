function UpdateGripperAndObject(robot, g1, g2, qCurrent, payload, vertices, holdingObject)

    object = ObjectClass();

    % Gripper base transform for UR3.
    pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0.0127,0,0.0612)*trotx(pi/2)*trotz(pi/2);%z0.0612
    pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(-0.0127,0,0.0612)*trotx(pi/2)*trotz(pi/2);%z0.0612
    
    g1.model.base = pos1; 
    g2.model.base = pos2; 
    g1.model.animate(g1.model.getpos());
    g2.model.animate(g2.model.getpos());
    
    % Update object position if holding.
    if holdingObject
        object.UpdateObjectPosition(robot, qCurrent, payload, vertices);
    end

end