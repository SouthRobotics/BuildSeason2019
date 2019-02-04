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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveForwardPID extends Command implements PIDOutput {
    private double pidOutput;
    private double targetDistance;
    private PIDController leftController;
    private PIDController rightController;
    private Encoder leftEncoder;        // Grayhill 63R
    private Encoder rightEncoder;
    private static final double wheelDiameter = 6;
    private static final double pulsesPerRevolution = 256;
    private static final double encoderGearRatio = 3;
    private static final double gearRatio = 64 / 1;
    private static final double distancePerPulse = Math.PI * wheelDiameter / pulsesPerRevolution / encoderGearRatio / gearRatio;

	public DriveForwardPID(double distance) {
        // Use requires() here to declare subsystem dependencies
        super("Drive Forward Distance PID");
        targetDistance = distance;
        //leftEncoder = RobotMap.leftEncoder;
        leftEncoder.setDistancePerPulse(distancePerPulse);
        //rightEncoder = RobotMap.rightEncoder;
        rightEncoder.setDistancePerPulse(distancePerPulse);
        requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        leftEncoder.reset();
        rightEncoder.reset();
        initPIDController();
    }
    
    public void initPIDController() {
        leftController = new PIDController(DriveTrain.Kp,
                DriveTrain.Ki,
                DriveTrain.Kd,
                leftEncoder, this);    //pid values need tuning, especially for smaller angles!
        leftController.setInputRange(-1000.0, 1000.0);
        leftController.setOutputRange(-0.6, 0.6);  // don't need to rotate extremely fast
        leftController.setAbsoluteTolerance(1);  // 2 degree threshold
        leftController.setContinuous(true);
        leftController.setSetpoint(targetDistance);
        leftController.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
                Robot.robotDrive.tankDrive(pidOutput, -pidOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (leftController.onTarget() && isTimedOut())
            return true;
        return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.robotDrive.tankDrive(0, 0);
        //anglecontroller.disable();
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
