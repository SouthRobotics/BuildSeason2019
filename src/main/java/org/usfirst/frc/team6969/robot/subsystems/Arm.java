/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import java.awt.Robot;

import javax.lang.model.util.ElementScanner6;

import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * 2019 build season
 */
public class Arm extends Subsystem {

    private static Spark rotatingPlatformMotor;
    private static Spark bottomMotor;
    private static Spark middleMotor;
    private static Spark topMotor;
    private static Potentiometer bottomPotentiometer;
    private static Potentiometer middlePotentiometer;
    private static Potentiometer topPotentiometer;
    private static Encoder rotatingPlatformEncoder;

    public void initDefaultCommand() {
        rotatingPlatformMotor = RobotMap.rotatingPlatformMotor;
        bottomMotor = RobotMap.bottomMotor;
        middleMotor = RobotMap.middleMotor;
        topMotor = RobotMap.topMotor;
        bottomPotentiometer = RobotMap.bottomPotentiometer;
        middlePotentiometer = RobotMap.middlePotentiometer;
        topPotentiometer = RobotMap.topPotentiometer;
        rotatingPlatformEncoder = RobotMap.rotatingPlatformEncoder;
        setDefaultCommand(null);
	}
    
    public void rotate(int joint, double speed) {  
        if (joint == 0)
            rotatingPlatformMotor.set(speed);
        else if (joint == 1)
            bottomMotor.set(speed);
        else if (joint == 2)
            middleMotor.set(speed);
        else
            topMotor.set(speed);
    }
}
