/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Roller intake 2019 build season
 */
public class Claw extends Subsystem {

    private static SpeedController leftMotor;
    private static SpeedController rightMotor;

    public void initDefaultCommand() {
        leftMotor = RobotMap.clawLeft;
        rightMotor = RobotMap.clawRight;
        setDefaultCommand(null);
	}
    
    public void spinIn() {
        leftMotor.set(-1);
        rightMotor.set(-1);
    }

    public void spinOut() {
        leftMotor.set(1);
        rightMotor.set(1);
    }
}
