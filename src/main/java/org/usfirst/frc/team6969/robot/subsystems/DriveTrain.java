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

public class DriveTrain extends Subsystem {

	private static DifferentialDrive robotDrive;
	private static boolean goHalfSpeed;
	private static boolean goFullSpeed;
	private static int leftYAxis;
	private static int rightYAxis;
	
    public void initDefaultCommand() {
    	robotDrive =  RobotMap.drive;
        goHalfSpeed = false;
        goFullSpeed = false;
        leftYAxis = Robot.m_oi.leftYAxis;
		rightYAxis = Robot.m_oi.rightYAxis;
		robotDrive =  RobotMap.drive;
        goHalfSpeed = false;
		goFullSpeed = false;
        setDefaultCommand(new TeleOpDrive());
    }	
}