/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import org.usfirst.frc.team6969.robot.subsystems.*;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TestMovementCommand;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;


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
//	public static TestMovement testMovement;
	
	//controller map
	public static OI m_oi;
	
    public static DifferentialDrive robotDrive;
	public static DriverStation ds;
	public static PowerDistributionPanel pdp;
	
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
//		testMovement = new TestMovement();
		m_oi = new OI();
		pdp = new PowerDistributionPanel(30);
		ds = DriverStation.getInstance();
		pdp.clearStickyFaults();	//clears pdp issue with yellow light
		robotDrive =  RobotMap.drive;
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
		DriverStation.Alliance color = ds.getAlliance();
		int station = ds.getLocation();
		
		//choose auto command based on location/team
		
		//testing this, not sure if it will work
		//TODO check if this works. Should execute a new instance of TestMovementCommans.java
		autonomousCommand = new TestMovementCommand(); //new auto(color, station)

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
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

