/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team6969.robot.OI;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.ClawDriveCommand;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class ClawDrive extends Subsystem {
  private static DifferentialDrive clawDrive;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
   clawDrive =  RobotMap.clawdrive;
   setDefaultCommand(new ClawDriveCommand());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void takeJoystickInputs(OI oi) {

    if ( clawDrive == null ) {
      this.initDefaultCommand();
    
    }
    if ( oi.circleButton.get() ){
          clawDrive.tankDrive(1, 1);
        }
    if ( oi.triangleButton.get() ){
          clawDrive.tankDrive(-1, -1);
        }
      }
  public void stop() {
    clawDrive.stopMotor();
}
}
