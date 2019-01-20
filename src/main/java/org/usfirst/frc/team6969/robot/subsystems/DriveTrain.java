/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.OI;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TeleOpDrive;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends Subsystem {

	private static DifferentialDrive robotDrive;
	private static boolean goHalfSpeed;
	private static boolean goFullSpeed;
	private static int leftYAxis;
	private static int rightYAxis;
	private static double error;
	private static double threshold;
	private static double kP;
	private static double rotation;
	private static long startTime;
	public static AHRS navx;
	public static Boolean rotateLeft;
	
    public void initDefaultCommand() {
    	robotDrive =  RobotMap.drive;
        goHalfSpeed = false;
        goFullSpeed = false;
        leftYAxis = Robot.m_oi.leftYAxis;
		rightYAxis = Robot.m_oi.rightYAxis;
		robotDrive =  RobotMap.drive;
    	navx = RobotMap.navx;
        goHalfSpeed = false;
		goFullSpeed = false;
		rotateLeft = false;
		error = 0.0;
		threshold = 0.5;
		kP = 0.1;
		rotation = 0.0;
		startTime = 0;
        setDefaultCommand(new TeleOpDrive());
    }
    
    public void takeJoystickInputs(OI oi) {

    	if ( robotDrive == null ) {
    		this.initDefaultCommand();
		}
		
    	//Speed Controls
    	if ( oi.leftBumper.get() )
    			goHalfSpeed = true;
    	if ( !oi.leftBumper.get() )
    			goHalfSpeed = false;
    	if ( oi.rightBumper.get() )
    			goFullSpeed = true;    		
    	if ( !oi.rightBumper.get() )
    			goFullSpeed = false;

    	//Sets motor speeds
    	if ( !goHalfSpeed && !goFullSpeed ) {
			// 75% speed
	    	robotDrive.tankDrive( oi.getController().getRawAxis(leftYAxis) * -1 * 0.75, oi.getController().getRawAxis(rightYAxis) * -1 *  0.75 );
    	}
    	if ( goHalfSpeed ) {
    		robotDrive.tankDrive( oi.getController().getRawAxis(leftYAxis) * -1 * 0.5, oi.getController().getRawAxis(rightYAxis) * -1 * 0.5 );
    	}
    	if ( goFullSpeed ) {
    		robotDrive.tankDrive( oi.getController().getRawAxis(leftYAxis) * -1 , oi.getController().getRawAxis(rightYAxis) * -1 );
    	}
    }
    
    //Arcade drive rather than take drive
    public void takeArcadeInputs(double speed, double zRotation) {
    	robotDrive.arcadeDrive(speed, zRotation);
    }
    
    public void stop() {
    	robotDrive.stopMotor();
	}
	
	/*
	Takes an angle -180 to 180 and rotates to that angle. Works with 0.5 degrees of precision and takes < 4.5 seconds for any angle.
	Pre: targetAngle is a double between -180 and 180
	Post: Robot turns chassis to proper angle
	*/
	public void rotateChassisToAngle(double targetAngle) {

        // For angles below 0, rotate to the left. For angles above 0, rotate right.
        if ( targetAngle < 0 ) 
            rotateLeft = true;
        else
			rotateLeft = false;
			
		navx.zeroYaw();	// set the direction we are facing right now as zero
		error = targetAngle - navx.getYaw(); // how far we are from target angle

		// Get current time. Don't want to get stuck in infinite loop if we are stuck against wall or something.
		startTime = System.currentTimeMillis();

		// Loop terminates target is reached or if haven't reached target in under 5 seconds (most likely stuck against something)
        while ( Math.abs(error) > threshold && ( System.currentTimeMillis() - startTime ) < 5000 ) {

			error = Math.abs( targetAngle - navx.getYaw() );	// Degrees remaining to reach target angle
			rotation =  error*kP;  // update motor speed

			if ( rotation > .55 )	// Don't need to turn extremely fast, so max at .55 for better accuracy
				rotation = 0.55;

			// .45 is min amount of power to move motors. want to slow down within final 20 degrees to ensure we don't overshoot target
			if ( rotation < 0.45 || error < 20 )
				rotation = 0.45;

            if (rotateLeft)
				robotDrive.tankDrive( -rotation , rotation );   // Turn left side backwards, right side forwards
			else
				robotDrive.tankDrive( rotation, - rotation );  // Turn left side forwards, right side backwards
			
			// terminate if robot stops rotating over the last 1 second (robot is probably stuck)
			// time substraction is to ensure this isn't first time through loop because then robot hasn't had enough time to begin rotating so isRotating() will be false
			if ( ( System.currentTimeMillis() - startTime ) > 2000 && !navx.isRotating() )
				startTime -= 10000;
        }
        rotation = 0;  //stop turning
	}
}