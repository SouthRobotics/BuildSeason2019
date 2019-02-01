package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;

public class moveBotSlow extends Command{

    private boolean done;
    private int count;

    public moveBotSlow() {
        requires(Robot.driveTrain);
    }
    @Override
    protected void initialize() {
        done = false;
        count = 0;
    }
    @Override
    protected void execute() {
        Robot.robotDrive.tankDrive(0.5,0.5);
        count++;
        if (count>300)
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

