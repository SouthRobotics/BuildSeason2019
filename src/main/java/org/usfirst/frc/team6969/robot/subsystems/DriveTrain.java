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
	
	public void initDefaultCommand() 
	{
    	robotDrive =  RobotMap.drive;
        setDefaultCommand(new TeleOpDrive());
	}	

	public boolean halfSpeed()//checks if left bumper is pressed -->go half speed
	{
		return Robot.m_oi.leftBumper.get();
	}
	public boolean fullSpeed()//checks if right bumper is pressed -->go full speed
	{
		return Robot.m_oi.rightBumper.get();
	}
	public double leftSpeed()//returns level that left trigger is pressed
	{
		return Robot.m_oi.getController().getRawAxis(Robot.m_oi.leftYAxis)*-1;//-1 because axis is inverted
	}
	public double rightSpeed()//returns level that right trigger is pressed
	{
		return Robot.m_oi.getController().getRawAxis(Robot.m_oi.rightYAxis)*-1;
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