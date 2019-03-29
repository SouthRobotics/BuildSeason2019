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

/**
 * An example command.  You can replace me with your own command.
 */
public class BallFromFloor extends Command {
	public int test = 0;
	public BallFromFloor() {
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Command servo = new OpenServo();
		System.out.println("bottom start" + Robot.bottomStart);
		System.out.println("middle start" + Robot.middleStart);
		System.out.println("top start" + Robot.topStart);
<<<<<<< HEAD
		//hard-coded presets: 101, 140, 215
		Command bottom = new LockJoint(RobotMap.bottomJointPot, Robot.arm.bottomAnglePID, 0, Robot.arm.bottomOut, 109 /*(Robot.bottomStart + 46)*/ );
		Command middle = new LockJoint(RobotMap.middleJointPot, Robot.arm.middleAnglePID, 1, Robot.arm.middleOut, 146 /*(Robot.middleStart + 20)*/ );
		Command top = new LockJoint(RobotMap.topJointPot, Robot.arm.topAnglePID, 2, Robot.arm.topOut, 215 /*(Robot.topStart - 102)*/ );
=======
		Command bottom = new LockJoint(RobotMap.bottomJointPot, Robot.arm.bottomAnglePID, 0, Robot.arm.bottomOut, 101);
		Command middle = new LockJoint(RobotMap.middleJointPot, Robot.arm.middleAnglePID, 1, Robot.arm.middleOut, 140);
		Command top = new LockJoint(RobotMap.topJointPot, Robot.arm.topAnglePID, 2, Robot.arm.topOut, 215);
>>>>>>> parent of 0d3c420... end comp day 1
		servo.start();
		bottom.start();
		middle.start();
		top.start();
		test = 1;

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (test == 1);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
