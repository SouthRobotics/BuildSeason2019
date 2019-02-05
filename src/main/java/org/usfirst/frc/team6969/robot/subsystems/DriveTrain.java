/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.TeleOpDrive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem implements PIDOutput {

	private static DifferentialDrive robotDrive;
	public static double Kp;
	public static double Ki;
	public static double Kd;
	private double leftSpeed;
	private double rightSpeed;
	public static PIDController angleController;
	public static AHRS navx;
	public static double rotateSpeed;
	
	public DriveTrain() {
		Kp = 0.1;
		Ki = 0.01;
		Kd = 0.2;
		leftSpeed = 0;
		rightSpeed = 0;
		navx = RobotMap.navx;
		robotDrive =  RobotMap.drive;
		initPIDController();
	}

	public void initPIDController() {
        angleController = new PIDController(Kp,
                Ki,
                Kd,
                navx, this);    //pid values need tuning, especially for smaller angles!
        angleController.setInputRange(-180.0, 180.0);
        angleController.setOutputRange(-0.6, 0.6); // don't need to rotate extremely fast
        angleController.setAbsoluteTolerance(1); // 2 degree threshold
        angleController.setContinuous(true);
		//angleController.setSetpoint(targetAngle);
		SmartDashboard.putData(angleController);
    }

	public void initDefaultCommand() 
	{
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

	@Override
	public void pidWrite(double output) {
		rotateSpeed = output;
	}
	
}