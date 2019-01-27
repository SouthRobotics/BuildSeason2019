/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;

public class TeleOpDrive extends Command {
	private boolean goFullSpeed;
	private boolean goHalfSpeed;

	public TeleOpDrive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		
		goFullSpeed = false;
		goHalfSpeed = true;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//Speed Controls
    	if ( Robot.m_oi.leftBumper.get() )
    			goHalfSpeed = true;
    	if ( !Robot.m_oi.leftBumper.get() )
    			goHalfSpeed = false;
    	if ( Robot.m_oi.rightBumper.get() )
    			goFullSpeed = true;    		
    	if ( !Robot.m_oi.rightBumper.get() )
    			goFullSpeed = false;

    	//Sets motor speeds
    	if ( !goHalfSpeed && !goFullSpeed ) {
			// 75% speed
	    	Robot.robotDrive.tankDrive( Robot.m_oi.getController().getRawAxis(Robot.m_oi.leftYAxis) * -1 * 0.75, Robot.m_oi.getController().getRawAxis(Robot.m_oi.rightYAxis) * -1 *  0.75 );
    	}
    	if ( goHalfSpeed ) {
    		Robot.robotDrive.tankDrive( Robot.m_oi.getController().getRawAxis(Robot.m_oi.leftYAxis) * -1 * 0.5, Robot.m_oi.getController().getRawAxis(Robot.m_oi.rightYAxis) * -1 * 0.5 );
    	}
    	if ( goFullSpeed ) {
    		Robot.robotDrive.tankDrive( Robot.m_oi.getController().getRawAxis(Robot.m_oi.leftYAxis) * -1 , Robot.m_oi.getController().getRawAxis(Robot.m_oi.rightYAxis) * -1 );
    	}
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
