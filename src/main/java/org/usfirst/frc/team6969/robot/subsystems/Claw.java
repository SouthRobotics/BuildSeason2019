/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Roller intake 2019 build season
 */
public class Claw extends Subsystem {

    private static SpeedController leftMotor;
    private static SpeedController rightMotor;
    private static Servo servo;

    public void initDefaultCommand() {
        leftMotor = RobotMap.clawLeft;
        rightMotor = RobotMap.clawRight;
        servo = RobotMap.servo;
        setDefaultCommand(null);
	}
    
    public void spinIn() {  //slower intake is better control so ball doesn't bounce out
    leftMotor.set(0.25);
    rightMotor.set(-0.25);
    }

    public void spinOut() {
        /*leftMotor.set(0.35);
        rightMotor.set(-0.35);*/
        leftMotor.set(-0.8);
        rightMotor.set(0.8);
    }

    public void stopSpinning() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void closeServo() {
        servo.set(0.55);
    }

    public void openServo() {
        servo.set(0.0);
    }

    public void openServoInHatch() {
        servo.set(0.40);
    }
}
