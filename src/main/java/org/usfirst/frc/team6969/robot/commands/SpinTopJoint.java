/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

public class SpinTopJoint extends Command {
	// do not make this variable static or else boolean will never change after first call
    private double speed;	

	public SpinTopJoint(boolean forward) {
        //requires(Robot.arm);
		this.speed = 0.2;
		if (!forward)
			speed = -0.2;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        Robot.arm.rotate(2, speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.arm.rotate(2, 0);
		Scheduler.getInstance().add(new LockJoint(RobotMap.topJointPot, Robot.arm.topAnglePID, 2, Robot.arm.topOut, RobotMap.topJointPot.get()));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
