/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team6969.robot.RobotMap;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class ClawDrive extends Subsystem {
  private static DifferentialDrive clawDrive;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  public void initDefaultCommand() {
   clawDrive =  RobotMap.clawdrive;
   
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void shoot() {
     clawDrive.tankDrive(1, 1);
  }
  public void intake() {
      clawDrive.tankDrive(-1, -1);
  }
  public void stop() {
      clawDrive.stopMotor();
  }
}
