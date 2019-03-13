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

/**
 * An example command.  You can replace me with your own command.
 */
public class FaceTowardsBall extends Command {
	public static final int centerX = 320/2;
	public static final int centerY = 240/2;
	public static final double minSpeed = 0.3;
	public static final double maxSpeed = 0.8;
	public static final int threshold = 10;//how many pixels off center is still acceptable; the distance from the center which the robot should stop rotating

	public int offX(){
		int avgPos = 0;
		double[] xPos = Robot.ballx;
		if(xPos.length == 0){
			System.out.println("/!\\ WARNING: NO BALL DETECTED /!\\");
			return 0;
		}
		for(int i = 0;i < xPos.length;i++){
			avgPos += xPos[i];
		}
		avgPos /= xPos.length;
		avgPos -= centerX;
		System.out.println("Off from center by " + avgPos + " pixels");
		return avgPos;
	}
	public FaceTowardsBall() {
		// Use requires() here to declare subsystem dependencies
		//requires(Robot.m_subsystem);

		double rightSpeed = 0;
		if(){

		}

		RobotMap.drive.tankDrive(leftSpeed, rightSpeed);
	}



	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
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
