package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;

public class moveBotFast extends Command{

    
    private boolean done;
    private int count;

    public moveBotFast() {
        requires(Robot.driveTrain);
    }
    @Override
    protected void initialize() {
        done = false;
        count = 0;
    }
    @Override
    protected void execute() {
        Robot.robotDrive.tankDrive(1,1);
        count++;
        if (count > 200)
            done = true;
    }

    @Override
    protected boolean isFinished() {
        return done;
    }

    @Override
	protected void end() {
    }
    
    @Override
	protected void interrupted() {
    }
    
}

