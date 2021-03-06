/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.custom_classes.CustomPIDOutput;
import org.usfirst.frc.team6969.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LockJoint extends Command {
        private PIDController anglecontroller;
        private int joint;
        private CustomPIDOutput out;
        private double outValue, setpoint;

        public LockJoint(Potentiometer potentiometer, PIDController controller, int joint, CustomPIDOutput out, double setpoint) {
            // Use requires() here to declare subsystem dependencies
            super("Lock Joint");
            anglecontroller = controller;
            this.joint = joint;
            this.out = out;
            this.setpoint = setpoint;
            //requires(Robot.arm);
	    }

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
                initPIDController();
        }
    
    public void initPIDController() {
        anglecontroller.setSetpoint(setpoint);
        anglecontroller.setAbsoluteTolerance(1);
        anglecontroller.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        outValue = out.outVal;
 
        Robot.arm.rotate(joint, -outValue);
        SmartDashboard.putNumber("Out Value", out.outVal);
        SmartDashboard.putData(anglecontroller);
        SmartDashboard.putNumber("target setpoint", anglecontroller.getSetpoint());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
        return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        //anglecontroller.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        end();
	}
}
