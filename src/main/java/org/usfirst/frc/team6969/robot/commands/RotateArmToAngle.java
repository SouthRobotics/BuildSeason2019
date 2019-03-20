/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.custom_classes.CustomPIDOutput;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * An example command. You can replace me with your own command.
 */
public class RotateArmToAngle extends Command {
        private double targetAngle;
        private PIDController anglecontroller;
        private int joint;
        private CustomPIDOutput out;
        private Potentiometer pot;

	public RotateArmToAngle(Potentiometer potentiometer, PIDController controller, int joint, CustomPIDOutput out, double angle) {
        // Use requires() here to declare subsystem dependencies
        super("Rotate Arm To Angle PID");
        targetAngle = angle;
        anglecontroller = controller;
        this.joint = joint;
        this.out = out;
        requires(Robot.arm);
        pot = potentiometer;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        initPIDController();
    }
    
    public void initPIDController() {
        anglecontroller.setSetpoint(targetAngle);
        anglecontroller.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
                Robot.arm.rotate(joint, -out.outVal);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
              return anglecontroller.onTarget();  
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
                //anglecontroller.disable();
                //Scheduler.getInstance().add(new LockJoint(pot, anglecontroller, joint, out));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
                end();
	}
}
