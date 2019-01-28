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


//import org.usfirst.frc.team6969.robot.GyroItg3200;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends Subsystem {
	private static DifferentialDrive robotDrive;
	private static boolean goHalfSpeed;
	private static boolean goFullSpeed;
	private static int leftYAxis;
	private static int rightYAxis;
//	private static TestMovementCommand test;//cheese solution

	//public static ADXL345_I2C accelerometer; 

	
    public void initDefaultCommand() {
    	robotDrive =  RobotMap.drive;
    	//accelerometer = RobotMap.accelerometer;
        goHalfSpeed = false;
        goFullSpeed = false;

//        test = new TestMovementCommand();
//        test.start();//cheese solution

		
    }
    
    public void takeJoystickInputs(OI oi) {

    	if(robotDrive == null ) // prevents robotDrive from being null
    	{
    		this.initDefaultCommand();
    	}
    	//Speed Controls
    	if(oi.leftBumper.get())
    			goHalfSpeed = true;
    	if(!oi.leftBumper.get())
    			goHalfSpeed = false;
    	if(oi.rightBumper.get())
    			goFullSpeed = true;    		
    	if(!oi.rightBumper.get())
    			goFullSpeed = false;
    	
    	
    	//Sets motor speeds
    	if(!goHalfSpeed && !goFullSpeed) { //going 0.75 speed (NORMAL)
	    	robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 * 0.75, oi.getController().getRawAxis(rightYAxis) * -1 *  0.75);
    	}
    	if(goHalfSpeed) {
    		robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 * 0.5, oi.getController().getRawAxis(rightYAxis) * -1 * 0.5);
    	}
    	if(goFullSpeed) {
    		robotDrive.tankDrive(oi.getController().getRawAxis(leftYAxis) * -1 , oi.getController().getRawAxis(rightYAxis) * -1);
    	}
    }
    
    //Arcade drive rather than take drive
    public void takeArcadeInputs(double speed, double zRotation) {
    	robotDrive.arcadeDrive(speed, zRotation);
    }
    
    public void stop() {
    	robotDrive.stopMotor();
    }
}