/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class MoveDistance extends Command {
    public double distanceToMove;
    public double speed = 0.3;
    public Encoder encoder;

	public MoveDistance(double distance) {
		// Use requires() here to declare subsystem dependencies
        //requires(Robot.m_subsystem);
        System.out.println("declaring encoder...");
        this.distanceToMove = distance;
        try{
            encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        // encoder.setDistancePerPulse(1);
            encoder.reset();
        }catch(Exception e){
            System.out.println(e.toString());
        }
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        RobotMap.drive.tankDrive(speed, speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
        try{
            return encoder.getDistance() > distanceToMove;
        }catch(Exception e){
            System.out.println(e.toString());
        }
		return true;
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
