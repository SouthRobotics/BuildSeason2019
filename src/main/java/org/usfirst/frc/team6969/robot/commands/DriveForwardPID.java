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
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveForwardPID extends Command implements PIDOutput {
        private double pidOutput;
        private double targetDistance;
        private PIDController leftController;
        private PIDController rightController;
        private Encoder leftEncoder; // Grayhill 63R
        private Encoder rightEncoder;
        private static final double wheelDiameter = 6;
        private static final double pulsesPerRevolution = 256;
        private static final double encoderGearRatio = 3;
        private static final double gearRatio = 64 / 1;
        private static final double distancePerPulse = Math.PI * wheelDiameter / pulsesPerRevolution / encoderGearRatio
                        / gearRatio;

        public DriveForwardPID(double distance) {
                // Use requires() here to declare subsystem dependencies
                super("Drive Forward Distance PID");
                targetDistance = distance;
                leftEncoder = RobotMap.leftDriveEncoder;
                leftEncoder.setDistancePerPulse(distancePerPulse);
                rightEncoder = RobotMap.rightDriveEncoder;
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
                PIDSource leftSource = new PIDSource() {

                        @Override
                        public void setPIDSourceType(PIDSourceType pidSource) {

                        }

                        @Override
                        public PIDSourceType getPIDSourceType() {
                                return PIDSourceType.kDisplacement;
                        }

                        @Override
                        public double pidGet() {
                                return leftEncoder.getDistance();
                        }

            };
            PIDSource rightSource = new PIDSource() {

                @Override
                public void setPIDSourceType(PIDSourceType pidSource) {

                }

                @Override
                public PIDSourceType getPIDSourceType() {
                        return PIDSourceType.kDisplacement;
                }

                @Override
                public double pidGet() {
                        return rightEncoder.getDistance();
                }

    };
    PIDOutput leftOut = new PIDOutput(){
    
            @Override
            public void pidWrite(double output) {
                    Robot.driveTrain.setLeftSpeed(output);
            }
    };
    PIDOutput rightOut = new PIDOutput(){
    
            @Override
            public void pidWrite(double output) {
                    Robot.driveTrain.setRightSpeed(output);
            }
    };
        leftController = new PIDController(DriveTrain.Kp,
                DriveTrain.Ki,
                DriveTrain.Kd,
                leftSource, leftOut);    //pid values need tuning, especially for smaller angles!
        leftController.setInputRange(-1000.0, 1000.0);
        leftController.setOutputRange(-0.6, 0.6);  // don't need to rotate extremely fast
        leftController.setAbsoluteTolerance(1);  // 2 degree threshold
        leftController.setContinuous(true);
        leftController.setSetpoint(targetDistance);
        leftController.enable();
        rightController = new PIDController(DriveTrain.Kp,
                DriveTrain.Ki,
                DriveTrain.Kd,
                rightSource, rightOut);    //pid values need tuning, especially for smaller angles!
        rightController.setInputRange(-1000.0, 1000.0);
        rightController.setOutputRange(-0.6, 0.6);  // don't need to rotate extremely fast
        rightController.setAbsoluteTolerance(1);  // 2 degree threshold
        rightController.setContinuous(true);
        rightController.setSetpoint(targetDistance);
        rightController.enable();
    }

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
                Robot.driveTrain.drive();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
	if (leftController.onTarget() && rightController.onTarget())
            return true;
        return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        Robot.robotDrive.tankDrive(0, 0);
        leftController.disable();
        rightController.disable();
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
