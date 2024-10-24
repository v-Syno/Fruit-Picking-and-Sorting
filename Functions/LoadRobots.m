function [harvesterBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots

    robotRaised = 0.05;
    
    % Initialize robots and their respective collision functions
    harvesterBot = LinearUR3e(transl(0.4,0.6,robotRaised));
    sortingBot = Panda(transl(0.75,-0.25,robotRaised) * trotz(pi));
   
    
    rightHarvesterPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(0.0127,0,0.0612)*trotx(pi/2)*troty(-pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
    leftHarvesterPos = (harvesterBot.model.fkineUTS(harvesterBot.model.getpos()))*transl(-0.0127,0,0.0612)*trotx(pi/2)*troty(-pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
    rightHarvester = GripRight(rightHarvesterPos); % initiate right gripper
    leftHarvester = GripLeft(leftHarvesterPos); % initial left gripper
    
    rightSorterPos = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))* transl(0,-0.0127,0.05)*trotx(-pi/2)*trotz(pi);  % Base position right gripper offset from Panda's end effector 
    leftSorterPos = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))* transl(0,0.0127,0.05)*trotx(-pi/2)*trotz(pi); % Base position left gripper offset from Panda's end effector 
    rightSorter = GripRight(rightSorterPos); % initiate right gripper
    leftSorter = GripLeft(leftSorterPos); % initial left gripper 
    
    RobotClass.GripperMove(rightHarvester,leftHarvester,'open');
    RobotClass.GripperMove(rightSorter,leftSorter,'open');

end