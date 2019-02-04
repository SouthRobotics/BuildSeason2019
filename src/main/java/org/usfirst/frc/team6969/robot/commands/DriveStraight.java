/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveStraight extends Command {
    private double error;
    private Encoder leftEncoder;    // Grayhill 63R
    private Encoder rightEncoder;
    private static final double speed = 0.6;
    private static final double wheelDiameter = 6;
    private static final double pulsesPerRevolution = 256;
    private static final double encoderGearRatio = 3;
    private static final double gearRatio = 64 / 1;
    private static final double distancePerPulse = Math.PI * wheelDiameter / pulsesPerRevolution / encoderGearRatio / gearRatio;
    private static final double kP = 0.05;

	public DriveStraight() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        leftEncoder = RobotMap.leftDriveEncoder;
        rightEncoder = RobotMap.rightDriveEncoder;
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
        leftEncoder.reset();
        rightEncoder.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        error = leftEncoder.getDistance() - rightEncoder.getDistance();
        Robot.robotDrive.arcadeDrive(speed, kP * error, false);
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
