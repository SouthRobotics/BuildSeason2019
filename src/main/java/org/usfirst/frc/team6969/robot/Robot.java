/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import java.util.ArrayList;

import org.usfirst.frc.team6969.robot.commands.LockJoint;
import org.usfirst.frc.team6969.robot.commands.TeleOpDrive;
import org.usfirst.frc.team6969.robot.custom_classes.ForwardKin;
import org.usfirst.frc.team6969.robot.subsystems.Arm;
import org.usfirst.frc.team6969.robot.subsystems.Claw;
import org.usfirst.frc.team6969.robot.subsystems.DriveTrain;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
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
	// Robot class controls the whole robot
	// if you ever get lost: https://frc-pdr.readthedocs.io/en/latest/index.html
	
	//subsystems
	//public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
	public static DriveTrain driveTrain;
	public static Claw claw;
	public static Arm arm;

	
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
	public static final double[] armLengths = {32.5, 32.25, 11};
	public static Command bottomLimit = new LockJoint(RobotMap.bottomJointPot, Robot.arm.bottomAnglePID, 0, Robot.arm.bottomOut, 64);
	public static double bottomStart, middleStart, topStart;

	
	//auto command... will vary based on location/alliance
	public Command autonomousCommand = null;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.init();		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(640, 480);
		camera.setFPS(15);
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
		camera1.setResolution(640, 480);
		camera1.setFPS(15);
		claw = new Claw();
		arm = new Arm();
		m_oi = new OI();
		driveTrain = new DriveTrain();
		pdp = new PowerDistributionPanel(30);
		ds = DriverStation.getInstance();
		pdp.clearStickyFaults();	// clears pdp issue with yellow light
		robotDrive =  RobotMap.drive;
		arduino = new SerialPort(9600, SerialPort.Port.kUSB);
		arduinoString = "";
		pixyData = new ArrayList<Integer>();
		for (int i = 0; i < 50; i++)	// initialize pixyData
			pixyData.add(new Integer(-1));
		pixyCenter = 158;
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
		resetEncoders();
		bottomStart = RobotMap.bottomJointPot.get();
		middleStart = RobotMap.middleJointPot.get();
		topStart = RobotMap.topJointPot.get();
		//for every subsystem just do subsystem.initDefaultCommand()
		//subsystems
		//driveTrain.initDefaultCommand();
		SmartDashboard.putData(arm.middleAnglePID);

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		getPixyData();
		System.out.println("Center: " + pixyCenter);
		reportCollisionDetection();
		displaySmartDashboardData();
		RobotMap.drive.feedWatchdog();
		//if (RobotMap.bottomJointPot.get() > 105)
			//bottomLimit.start();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void resetEncoders() {
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
		else
		    System.out.println("no arduino comm");
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

	public double getArmY() {
		return ForwardKin.getY(RobotMap.bottomJointPot.get(), RobotMap.middleJointPot.get() - 187, RobotMap.topJointPot.get() - 193 + 180, armLengths) - 12.5;
	}

	/*
	Place values to monitor in Smart Dashboard
	*/
	private void displaySmartDashboardData() {
		SmartDashboard.putBoolean("Robot is moving", RobotMap.navx.isMoving());
		SmartDashboard.putNumber("Yaw", RobotMap.navx.getYaw());
		SmartDashboard.putBoolean("Hatch Limit Switch", RobotMap.hatchLimitSwitch.get());
		SmartDashboard.putBoolean("Ball Limit Switch", RobotMap.ballLimitSwitch.get());
		SmartDashboard.putBoolean("rotating platform Limit Switch", RobotMap.rotatingPlatformLimitSwitch.get());
		SmartDashboard.putNumber("Bottom potentiometer", RobotMap.bottomJointPot.get());
		SmartDashboard.putNumber("Middle potentiometer", RobotMap.middleJointPot.get());
		SmartDashboard.putNumber("Top potentiometer", RobotMap.topJointPot.get());
		SmartDashboard.putNumber("bottom out", Arm.bottomOut.outVal);
		SmartDashboard.putData(Arm.bottomAnglePID);
		SmartDashboard.putData(Arm.middleAnglePID);
		SmartDashboard.putData(Arm.topAnglePID);
		SmartDashboard.putNumber("How far out", getArmY());
	}
}

