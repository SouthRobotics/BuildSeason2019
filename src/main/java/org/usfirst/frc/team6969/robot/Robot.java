/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team6969.robot.subsystems.Claw;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	private static final int IMG_WIDTH = 640;
	private static final int IMG_HEIGHT = 480;
	public static double[] ballx;
	public static double[] hatchx;
	public static double[] bally;
	public static double[] hatchy;
	NetworkTableEntry centerball;
	NetworkTableEntry centerhatch;
	NetworkTableEntry centerbally;
	NetworkTableEntry centerhatchy;


	
	
	// Robot class controls the whole robot
	// if you ever get lost: https://frc-pdr.readthedocs.io/en/latest/index.html
	
	//subsystems
	//public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
	public static DriveTrain driveTrain;
	public static Claw claw;

	
	//controller map
	public static OI m_oi;
    public static DifferentialDrive robotDrive;
	public static DriverStation ds;
	public static PowerDistributionPanel pdp;
	public static SerialPort arduino;
	private String arduinoString;
	private ArrayList<Integer> pixyData;
	private Integer pixyVal;
	private int pixyCounter;
	public static int pixyCenter;
	public static final int PIXYXCENTER = 158;	// pixy cam x-values range from 0 to 316 
	
	//auto command... will vary based on location/alliance
	public Command autonomousCommand = null;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.init();
		driveTrain = new DriveTrain();
		claw = new Claw();
		m_oi = new OI();
		pdp = new PowerDistributionPanel(30);
		ds = DriverStation.getInstance();
		//pdp.clearStickyFaults();	// clears pdp issue with yellow light
		robotDrive =  RobotMap.drive;
		//arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
		//arduinoString = "";
		//pixyData = new ArrayList<Integer>();
		//for (int i = 0; i < 50; i++)	// initialize pixyData
		//	pixyData.add(new Integer(-1));
		//pixyCenter = 158;
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		//UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
		//camera1.setResolution(IMG_WIDTH, IMG_HEIGHT);
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTableInstance inst1 = NetworkTableInstance.getDefault();
		NetworkTableInstance inst2 = NetworkTableInstance.getDefault();
		NetworkTableInstance inst3 = NetworkTableInstance.getDefault();
		
		inst.startClient();
		inst1.startClient();
		inst2.startClient();
		inst3.startClient();
		
		NetworkTable Ball = inst1.getTable("GRIP/BallReport");
		NetworkTable Hatch = inst.getTable("GRIP/HatchReport");
		NetworkTable Ball1 = inst2.getTable("GRIP/BallReport");
		NetworkTable Hatch1 = inst3.getTable("GRIP/HatchReport");

		centerball = Ball.getEntry("centerX");
		centerhatch = Hatch.getEntry("centerX");
		centerbally = Ball1.getEntry("centerY");
		centerhatchy = Hatch1.getEntry("centerY");
		
    	
		
    	
    	
    }

	
		

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {

		autonomousCommand = null;

		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		//for every subsystem just do subsystem.initDefaultCommand()
		//subsystems
		//driveTrain.initDefaultCommand();
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//getPixyData();
		//System.out.println("Center: " + pixyCenter);
		reportCollisionDetection();
		displaySmartDashboardData();
		ballx = centerball.getDoubleArray(ballx); 
		hatchx = centerhatch.getDoubleArray(hatchx); 
		bally = centerbally.getDoubleArray(bally); 
		hatchy = centerhatchy.getDoubleArray(hatchy); 

		String hatchString = hatchx.length>0?hatchx[0] + "":"None";
		String ballString = ballx.length>0?ballx[0] + "":"None";
		String hatchString1 = hatchy.length>0?hatchy[0] + "":"None";
		String ballString1 = bally.length>0?bally[0] + "":"None";

		System.out.println("HATCH: (" + hatchString + ", " + hatchString1 + ") BAll: (" + ballString + ", " + ballString1 + ")");
	
		
		
		
		
		
	}
		
	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	/*
	Communicates with arduino to receive x-coordinate of center of target and store in array
	Pre: Arduino is connected to RoboRio via usb port 1
	Post: x-coordinate of center of target is stored in pixyCenter
	*/
	private void getPixyData() {

		arduino.write(new byte[] {0x12}, 1);	//RoboRio must initiate communication with arduino
		
		if ( arduino.getBytesReceived() > 0 ) {

			arduinoString = arduino.readString();

			if ( pixyData.size() > 50 )	// only remember last 3 centers
				pixyData.remove(0);

			try{	//arduino sometimes passes strings containing characters other than #s when detection not found
				
				pixyVal = new Integer(Integer.parseInt(arduinoString));

				if ( pixyVal.intValue() >= 0 ) {	//-1 is default if no object detected			
					pixyData.add(pixyVal);
					pixyCounter = 0;
				}
				else
					pixyCounter++;
			}
			catch(Exception e){
				System.out.println("Error parsing Pixy data.");
			}

			pixyCenter = pixyData.get(49);

			if ( pixyCounter > 8 )	//if lost sight of object stop turning
				pixyCenter = -1;
		}
	}

	/*
	Sends error message to Driver Station warning that a collision may have occured
	*/
	private void reportCollisionDetection() {
		// code taken from navx website to detect if robot has crashed by looking for a "jerk"
		// https://pdocs.kauailabs.com/navx-mxp/examples/collision-detection/
		
		double last_world_linear_accel_x = 0;
  		double last_world_linear_accel_y = 0;
		boolean collisionDetected = false;
		final double kCollisionThreshold_DeltaG = 1.5f;
          
        double curr_world_linear_accel_x = RobotMap.navx.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = RobotMap.navx.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
          
        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
             ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
            collisionDetected = true;
		}
		
		if ( collisionDetected )
			DriverStation.reportError("COLLISION DETECTED", false);

	}

	/*
	Place values to monitor in Smart Dashboard
	*/
	private void displaySmartDashboardData() {
		SmartDashboard.putBoolean("Robot is moving", RobotMap.navx.isMoving());
		SmartDashboard.putNumber("Yaw", RobotMap.navx.getYaw());
		//SmartDashboard.putNumber("potentiometer", RobotMap.pot.get());
	}
}

