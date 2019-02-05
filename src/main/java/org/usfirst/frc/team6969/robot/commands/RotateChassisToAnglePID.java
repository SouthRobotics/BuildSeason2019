/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

/**
 * An example command.  You can replace me with your own command.
 */
public class RotateChassisToAnglePID extends Command {
    private double targetAngle;
    public static PIDController anglecontroller;
    private AHRS navx;
    private int count;

	public RotateChassisToAnglePID(double angle) {
        // Use requires() here to declare subsystem dependencies
        super("Rotate Chassis To Angle PID");
        targetAngle = angle;
        navx = RobotMap.navx;
        requires(Robot.driveTrain);
        anglecontroller = DriveTrain.angleController;
        count = 0;
        }

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
                navx.zeroYaw();
                anglecontroller.setSetpoint(90);
                anglecontroller.enable();
        }
    
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
                Robot.robotDrive.tankDrive(DriveTrain.rotateSpeed, -1*DriveTrain.rotateSpeed);
        }

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
                return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
                Robot.robotDrive.tankDrive(0, 0);
                anglecontroller.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
                end();
	}
}
