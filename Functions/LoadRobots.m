function [harvestBot,sortingBot,rightHarvester,leftHarvester,rightSorter,leftSorter] = LoadRobots

    % Initialize robots and their respective collision functions
    sortingBot = LinearUR3e(transl(0.6,-0.2,0.4) * trotz(pi/2));
    harvestBot = Panda(transl(0,0.9,0.05) * trotz(pi));
   
    
    rightSorter = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))*transl(0.0127,0,0.0612)*trotx(pi/2)*troty(-pi/2);
    leftSorter = (sortingBot.model.fkineUTS(sortingBot.model.getpos()))*transl(-0.0127,0,0.0612)*trotx(pi/2)*troty(-pi/2); 
    rightSorter = GripRight(rightSorter); % initiate right gripper
    leftSorter = GripLeft(leftSorter); % initial left gripper
    
    rightHarvesterPos = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))* transl(0,-0.0127,0.05)*trotx(-pi/2)*trotz(pi);  % Base position right gripper offset from Panda's end effector 
    leftHarvesterPos = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))* transl(0,0.0127,0.05)*trotx(-pi/2)*trotz(pi); % Base position left gripper offset from Panda's end effector 
    rightHarvester = GripRight(rightHarvesterPos); % initiate right gripper
    leftHarvester = GripLeft(leftHarvesterPos); % initial left gripper 
    
    RobotClass.GripperMove(rightSorter,leftSorter,'open');
    RobotClass.GripperMove(rightHarvester,leftHarvester,'open');

end