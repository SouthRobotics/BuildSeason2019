/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TeleOpDrive;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain extends Subsystem {

	private static DifferentialDrive robotDrive;
	public static double Kp;
	public static double Ki;
	public static double Kd;
	private double leftSpeed;
	private double rightSpeed;
	
	public DriveTrain() {
		Kp = 0.02;
		Ki = 0.0;
		Kd = 0.01;
		leftSpeed = 0;
		rightSpeed = 0;
	}

	public void initDefaultCommand() 
	{
    	robotDrive =  RobotMap.drive;
        setDefaultCommand(new TeleOpDrive());
	}	

	public boolean halfSpeed()//checks if left bumper is pressed -->go half speed
	{
		return Robot.m_oi.lButton1.get();
	}
	public boolean fullSpeed()//checks if right bumper is pressed -->go full speed
	{
		return Robot.m_oi.rButton1.get();
	}
	public double leftSpeed()//returns level that left trigger is pressed
	{
		return Robot.m_oi.leftJoy.getRawAxis(Robot.m_oi.yAxis)*-1;//-1 because axis is inverted
	}
	public double rightSpeed()//returns level that right trigger is pressed
	{
		return Robot.m_oi.rightJoy.getRawAxis(Robot.m_oi.yAxis)*-1;
	}
	public void setLeftSpeed(double speed) {
		leftSpeed = speed;
	}
	public void setRightSpeed(double speed) {
		rightSpeed = speed;
	}
	public void drive() {
		robotDrive.tankDrive(leftSpeed, rightSpeed);
	}
	public void move()//called in TeleOpDrive.java
	{
		if(halfSpeed())//multiplies by .5 if halfSpeed
			robotDrive.tankDrive(.5*leftSpeed(), .5*rightSpeed());
		else if(fullSpeed())
			robotDrive.tankDrive(leftSpeed(),rightSpeed());
		else
			robotDrive.tankDrive(.75*leftSpeed(), .75*rightSpeed());
	}
	
}