/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

/**
 * An example command.  You can replace me with your own command.
 */
public class RotateArmToAngle extends Command implements PIDOutput {
    private double pidOutput;
    private double targetAngle;
    private Potentiometer potentiometer;
    private PIDController anglecontroller;
    private int joint;

	public RotateArmToAngle(double angle, Potentiometer potentiometer, int joint) {
        // Use requires() here to declare subsystem dependencies
        super("Rotate Arm To Angle PID");
        targetAngle = angle;
        this.potentiometer = potentiometer;
        this.joint = joint;
        requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        targetAngle += potentiometer.get();
        initPIDController();
    }
    
    public void initPIDController() {
        anglecontroller = new PIDController(DriveTrain.Kp,
                DriveTrain.Ki,
                DriveTrain.Kd,
                potentiometer, this);    //pid values need tuning, especially for smaller angles!
        anglecontroller.setInputRange(0.0, 360.0);
        anglecontroller.setOutputRange(-0.6, 0.6);  // don't need to rotate extremely fast
        anglecontroller.setAbsoluteTolerance(1);  // 2 degree threshold
        anglecontroller.setContinuous(true);
        anglecontroller.setSetpoint(targetAngle);
        anglecontroller.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        Robot.arm.rotate(joint, pidOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (anglecontroller.onTarget())
            return true;
        return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.arm.rotate(joint, 0);
        anglecontroller.disable();
        Scheduler.getInstance().add(new LockJoint(potentiometer, joint));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
        end();
	}

    @Override
    public void pidWrite(double output) {
        pidOutput = output;
    }
}
