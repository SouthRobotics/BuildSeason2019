/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;


public class RotatePotToAngle extends Command {
    private double error;
	private double threshold;
	private double kP;
	private double rotation;
    private Boolean rotateNeg;
    private double targetAngle;
    private double currentAngle;
    private Potentiometer pot;
    private Spark motor;

	public RotatePotToAngle(Potentiometer pot, Spark motor, double angleToRotate) {
        //requires(Robot.arm);
        targetAngle = angleToRotate;
        kP = 0.03;
        rotation = 0;
        threshold = 1;
        this.pot = pot;
        this.motor = motor;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        currentAngle = pot.get();
        error = targetAngle - currentAngle;

        if ( error < 0 )
            rotateNeg = true;
        else
            rotateNeg = false;

        if ( Math.abs(error) > threshold ) {

            rotation = Math.abs(error) * kP; // update motor speed

            if (rotation > .55) // Don't need to turn extremely fast, so max at .55 for better accuracy
                rotation = 0.55;

            // .45 is min amount of power to move motors. want to slow down within final 20
            // degrees to ensure we don't overshoot target
            if (rotation < 0.45 || error < 20)
                rotation = 0.45;

            if (rotateNeg)
                motor.set( -1 * rotation );
            else
                motor.set( rotation );
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
        motor.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        motor.set(0);
	}
}
