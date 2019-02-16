/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RotatingPlatform extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static SpeedController motor;
  public static Encoder enc;
  private double ratio;
  private double pos;

  public RotatingPlatform()
  {
    motor = RobotMap.platformMotor;
    enc = RobotMap.platformEncoder;
    ratio = 4*254.0/18;

  }

  public void move(double newPos)
  {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
