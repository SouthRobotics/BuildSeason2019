/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

public class DriveDistance extends Command {
    private static final double TICKS_PER_INCH = 40.5;
    private static final double POWER = 0.;
    private double targetDistance;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private AHRS navx;
    private double initialAngle;
    private double initialEncoderReading;
    private double[] pow;

	public DriveDistance(double targetDistance) {   //targetDistance in inches
        requires(Robot.driveTrain);
        this.targetDistance = targetDistance;
        navx = RobotMap.navx;
        leftEncoder = RobotMap.leftDriveEncoder;
        rightEncoder = RobotMap.rightDriveEncoder;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        initialAngle = navx.getYaw();
        initialEncoderReading = leftEncoder.get();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        pow = driveStraight(POWER, navx.getYaw()-initialAngle, 0.005);
		Robot.driveTrain.drive(-pow[0], -pow[1]);	
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Math.abs(leftEncoder.get()-initialAngle)>=Math.abs(targetDistance * TICKS_PER_INCH);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.driveTrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        end();
    }
    
    public static double[] driveStraight(double powSetpoint, double angleDifference, double tuningConstant) {
		return new double[] {(powSetpoint + (angleDifference*tuningConstant)), (powSetpoint - (angleDifference*tuningConstant))};
	}
}
