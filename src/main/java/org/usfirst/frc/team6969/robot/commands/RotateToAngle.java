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

import com.kauailabs.navx.frc.AHRS;

public class RotateToAngle extends Command {
    private double error;
	private double threshold;
	private double kP;
	private double rotation;
	private AHRS navx;
    private Boolean rotateLeft;
    private double targetAngle;

    public RotateToAngle(double angle) {
        requires(Robot.driveTrain);

        targetAngle = angle;
        navx = RobotMap.navx;
        navx.zeroYaw(); // set the direction we are facing right now as zero
		rotateLeft = false;
		error = 0.0;
		threshold = 0.5;
		kP = 0.1;
		rotation = 0.0;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        navx.zeroYaw();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        /*
         * Takes an angle -180 to 180 and rotates to that angle. Works with 0.5 degrees
         * of precision and takes < 4.5 seconds for any angle. Pre: targetAngle is a
         * double between -180 and 180 Post: Robot turns chassis to proper angle
         */
        // For angles below 0, rotate to the left. For angles above 0, rotate right.
        if (targetAngle < 0)
            rotateLeft = true;
        else
            rotateLeft = false;

        error = targetAngle - navx.getYaw(); // how far we are from target angle

        if ( Math.abs(error) > threshold ) {

            error = Math.abs(targetAngle - navx.getYaw()); // Degrees remaining to reach target angle
            rotation = error * kP; // update motor speed

            if (rotation > .55) // Don't need to turn extremely fast, so max at .55 for better accuracy
                rotation = 0.55;

            // .45 is min amount of power to move motors. want to slow down within final 20
            // degrees to ensure we don't overshoot target
            if (rotation < 0.45 || error < 20)
                rotation = 0.45;

            if (rotateLeft)
                Robot.robotDrive.tankDrive(-rotation, rotation); // Turn left side backwards, right side forwards
            else
                Robot.robotDrive.tankDrive(rotation, -rotation); // Turn left side forwards, right side backwards
        }
        else {
            rotation = 0; // stop turning
            error = 0;
            isFinished();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if ( error == 0 )
            return true;
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