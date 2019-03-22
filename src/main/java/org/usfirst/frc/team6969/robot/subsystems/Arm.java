/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;

import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.custom_classes.*;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * 2019 build season
 */
public class Arm extends Subsystem{

    private static SpeedController rotatingPlatformMotor;
    private static SpeedController bottomMotor;
    private static SpeedController middleMotor;
    private static SpeedController topMotor;
    private static Potentiometer bottomPotentiometer;
    private static Potentiometer middlePotentiometer;
    private static Potentiometer topPotentiometer;
    private static Encoder rotatingPlatformEncoder;
    public static PIDController rotatingPlatformPID;
    public static PIDController bottomAnglePID;
    public static PIDController middleAnglePID;
    public static PIDController topAnglePID;
    public static CustomPIDOutput rotatingPlatformOut;
    public static CustomPIDOutput bottomOut;
    public static CustomPIDOutput middleOut;
    public static CustomPIDOutput topOut;


    private double[][] pidVals = {{0.04,0.0,0.03, 0},
                            {0.04,0.0,0.03, 0},
                            {0.01,0,0, 0}};

    public Arm()
    {
        rotatingPlatformMotor = RobotMap.rotatingPlatformMotor;
        bottomMotor = RobotMap.bottomJointMotor;
        middleMotor = RobotMap.middleJointMotor;
        topMotor = RobotMap.topJointMotor;
        bottomPotentiometer = RobotMap.bottomJointPot;
        middlePotentiometer = RobotMap.middleJointPot;
        topPotentiometer = RobotMap.topJointPot;
        rotatingPlatformEncoder = RobotMap.rotatingPlatformEncoder;
        initPIDControllers();
    }
    
    public void initDefaultCommand() {

        setDefaultCommand(null);
    }
    
    public void initPIDControllers() 
    {
        rotatingPlatformOut = new CustomPIDOutput();
        bottomOut = new CustomPIDOutput();
        middleOut = new CustomPIDOutput();
        topOut = new CustomPIDOutput();

       /* rotatingPlatformPID = new PIDController(pidVals[0][0], pidVals[0][1], pidVals[0][2], pidVals[0][3], Robot., rotatingPlatformOut);    //pid values need tuning, especially for smaller angles!
        rotatingPlatformPID.setInputRange(0.0, 360.0);
        rotatingPlatformPID.setOutputRange(-0.3, 0.3);
        rotatingPlatformPID.setAbsoluteTolerance(4); 
        rotatingPlatformPID.setContinuous(false);*/
        
        bottomAnglePID = new PIDController(pidVals[0][0], pidVals[0][1], pidVals[0][2], pidVals[0][3], bottomPotentiometer, bottomOut);    //pid values need tuning, especially for smaller angles!
        bottomAnglePID.setInputRange(0.0, 360.0);
        bottomAnglePID.setOutputRange(-0.3, 0.3);  // don't need to rotate extremely fast
        bottomAnglePID.setAbsoluteTolerance(1);  // 2 degree threshold
        bottomAnglePID.setContinuous(false);

        middleAnglePID = new PIDController(pidVals[1][0], pidVals[1][1], pidVals[1][2], pidVals[1][3], middlePotentiometer, middleOut);    //pid values need tuning, especially for smaller angles!
        middleAnglePID.setInputRange(0.0, 360.0);
        middleAnglePID.setOutputRange(-0.25, 0.25);  // don't need to rotate extremely fast
        middleAnglePID.setAbsoluteTolerance(1);  // 2 degree threshold
        middleAnglePID.setContinuous(false);

        topAnglePID = new PIDController(pidVals[2][0], pidVals[2][1], pidVals[2][2], pidVals[2][3], topPotentiometer, topOut);    //pid values need tuning, especially for smaller angles!
        topAnglePID.setInputRange(0, 360.0);
        topAnglePID.setOutputRange(-0.2, 0.2);  // don't need to rotate extremely fast
        topAnglePID.setAbsoluteTolerance(1);  // 2 degree threshold
        topAnglePID.setContinuous(false);
    }
    
    public void rotate(int joint, double speed) {  
        if (joint == 0)
            bottomMotor.set(speed);
        else if (joint == 1)
            middleMotor.set(speed);
        else if (joint == 2)
            topMotor.set(speed);
    }
}
