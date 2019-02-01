package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.Robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;

public class MoveToTarget extends Command{

    private static int distanceToGo;    

    @Override
    protected void initialize() {
        requires(Robot.driveTrain);
        distanceToGo = 0;
    }

    protected void execute() {
        //distanceToGo = ultarsonic distance
       // if(distance)
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

}