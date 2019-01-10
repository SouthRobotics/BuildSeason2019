/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//All imports below are the default imports that come with the FRC package
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
//hhjhjh
public class RobotMap {
	//RobotMap class connects hardware to software based on the port # the hardware is plugged into on the PDP.

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	
	//Basic TankDrive drivetrain from KOP
	public static double driveWheelRadius = 3.0; //inches
	public static double driveWheelDiameter = driveWheelRadius * 2; //6 inches
	public static double driveWheelCircumferance = driveWheelRadius * 2 * Math.PI; // 18.84956inches
	public static double wheelRotation360degree = 6.97814; //amount of wheel rotations the robot will make 360* turn in when both sides are driving opposite ways 
	
	//Talons that control drivetrain
    public static SpeedController driveTrainLeftFront;	//must declare as type SpeedController
    public static SpeedController driveTrainLeftBack;
    public static SpeedController driveTrainRightFront;
    public static SpeedController  driveTrainRightBack;
    
    //Spark motorcontrollers that control subsystems
    
    //list of sensors: http://1418.team/assets/resources/Introduction%20to%20Sensors.pdf
    
    //Limit switches
    public static DigitalInput magLimit;
    /*
     * Example code to use limit switch:
     * if (limitSwitchName.get())
     * 	motor.setSpeed(0);
     */
    
    //Other sensors
    public static AnalogGyro gyro; //gyro documentation: http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/ADXRS450_Gyro.html
    /*
     * Example code to use gyro:
     * gyro.calibrate() right when robot turns on (calibrate needs to work while robot is stationary)
     * in auto/teleop init functions, first line is gyro.reset(); This sets the current orientation of the robot as 0 degrees.
     * gyro.getAngle(); gets total degrees (continuous number that can exceed 360) change in robot orientation from last reset() call
     * to drive straight: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599713-gyros-measuring-rotation-and-controlling-robot-driving-direction
     */
    
    //Class for KOP basic drivetrain
	public static DifferentialDrive drive = null;
	
	
	public static void init() {
		//Talon Documentation: https://www.ctr-electronics.com/downloads/api/java/html/index.html?com/ctre/phoenix/motorcontrol/can/WPI_TalonSRX.html
		
		
		/*
		Current PDP port numbers for Talons:
		Talon 12 - Back Right
		Talon 13 - Front Right
		Talon 14 - Front Left
		Talon 15 - Back Left
		*/
		driveTrainLeftBack = new WPI_TalonSRX(15); 
		driveTrainLeftFront =  new WPI_TalonSRX(14);
		driveTrainRightBack = new WPI_TalonSRX(13); 
		driveTrainRightFront =  new WPI_TalonSRX(12);
		
		//PWM port numbers for subsystems.
		
		/*
		 * LiveWindow.add() adds a device to the smartdashboard (in DriverStation go to third tab on left, and change dashboard
		 * to smartdashboard) so that you can see more data about that device
		 */

	    //PWM ports for sensors
	    magLimit = new  DigitalInput(0);
	    
	    //Analog channels for sensors
		gyro = new AnalogGyro(1);
		ahrs = new AHRS(SerialPort.Port.kUSB); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
	    
	    //creates motor groups for TankDrive
	    final SpeedControllerGroup m_left = new SpeedControllerGroup(driveTrainLeftFront, driveTrainLeftBack); //left drivetrain motors
	    final SpeedControllerGroup m_right = new SpeedControllerGroup(driveTrainRightFront, driveTrainRightBack); //Right drivetrain motors
	    //creates TankDrive drivetrain	
	    drive = new DifferentialDrive(m_left, m_right);
	    
	    /*
	     * when true, safetyenabled turns off motors if their output isn't updated for a certain amount of time
	     * setExpiration() sets the time for safetyenabled
	     */
	}
}
