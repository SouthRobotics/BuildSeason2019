/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.custom_classes.CustomPIDOutput;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateToPixy extends Command {
        private PIDController anglecontroller;
        private CustomPIDOutput out;
        private double outValue;

        public RotateToPixy(PIDController controller, CustomPIDOutput out) {
            super("Rotate To Pixy Target");
            anglecontroller = controller;
            this.out = out;
	    }

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
                initPIDController();
        }
    
    public void initPIDController() {
        anglecontroller.setSetpoint(Robot.pixyCenter);
        anglecontroller.setAbsoluteTolerance(3);
        anglecontroller.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        anglecontroller.setSetpoint(Robot.pixyCenter);
        outValue = out.outVal;
 
        RobotMap.rotatingPlatformMotor.set(-outValue);
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
