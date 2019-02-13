/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.Robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;

public class RotateToHATCHTarget extends Command {
  
  public static SerialPort arduino;
  public static double center;
  public static int cen;
  public static final int HATCHCENTER = 320;     // Pixycam x values go from 0 to 316 (center is 158)
  private double error;
  private double rotation;
  private double kP;

  public RotateToHATCHTarget(int cen) {
    requires(Robot.driveTrain);
    center = cen;
    error = 0;
    rotation = 0;
    kP = 0.06;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    center = -1;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    center = Robot.ballx;

    // threshold of 10 x-coordinates on PixyCam
    if ( !( center > ( HATCHCENTER - 10 ) && center < ( HATCHCENTER + 10 ) ) ) {

      if ( center > 0 ) { // center is set to -1 when target is lost for 8 straight data packets from arduino

        error = Math.abs( HATCHCENTER - center );
        rotation = error * kP;

        if ( rotation > .55 )	// Don't need to turn extremely fast, so max at .55 for better accuracy
          rotation = 0.55;
    
        // .45 is min amount of power to move motors. want to slow down as we near target to ensure we don't overshoot
        if ( rotation < 0.45 || error < 50 )
          rotation = 0.35;

        if ( center > HATCHCENTER )
          Robot.robotDrive.tankDrive(rotation, -rotation);
        else
          Robot.robotDrive.tankDrive(-rotation, rotation);
      }
      else {  // target was lost
        Robot.pixyCenter = HATCHCENTER;
        isFinished(); // ends command so we don't endlessly rotate
      }
    }
    else {  // target was reached
      Robot.pixyCenter = HATCHCENTER;
      isFinished(); // ends command so we don't endlessly rotate
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (center == HATCHCENTER)
      return true;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
