/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.usfirst.frc.team6969.robot.subsystems.*;
import org.usfirst.frc.team6969.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import java.util.ArrayList;;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	//Robot class controls the whole robot
	//if you ever get lost: https://frc-pdr.readthedocs.io/en/latest/index.html
	
	//subsystems
	//public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
	public static DriveTrain driveTrain;

	
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
		m_oi = new OI();
		pdp = new PowerDistributionPanel(30);
		ds = DriverStation.getInstance();
		pdp.clearStickyFaults();	// clears pdp issue with yellow light
		robotDrive =  RobotMap.drive;
		arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
		arduinoString = "";
		pixyData = new ArrayList<Integer>();
		for (int i = 0; i < 3; i++)	// initialize pixyData
			pixyData.add(new Integer(PIXYXCENTER));
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
		driveTrain.initDefaultCommand();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		getPixyData();
		reportCollisionDetection();
		displaySmartDashboardData();
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

			if ( pixyData.size() > 3 )	// only remember last 3 centers
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

			pixyCenter = pixyData.get(0);

			if ( pixyCounter > 8 )	//if lost sight of object stop turning
				pixyCenter = PIXYXCENTER;
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
	}
}

