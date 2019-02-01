/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

public class SpinRollerIntake extends Command {
	// do not make this variable static or else boolean will never change after first call
    private boolean spinIn;	

	public SpinRollerIntake(boolean spinInwards) {
        requires(Robot.claw);
        spinIn = spinInwards;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        if (spinIn)
            Robot.claw.spinIn();
        else
            Robot.claw.spinOut();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		//if ( ! (Robot.m_oi.leftBumper.get() || Robot.m_oi.rightBumper.get() ) )
	      //  return true;
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.claw.stopSpinning();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.claw.stopSpinning();
	}
}
