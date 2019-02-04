/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class MoveClawToHeight extends Command {
	public static final double FIRST_HEIGHT = 27.5;//inches
	public static final double DISTANCE_BETWEEN_PORTS = 28;//inches
	public static final double HEIGHT_1 = FIRST_HEIGHT;//height of first port
	public static final double HEIGHT_2 = FIRST_HEIGHT + DISTANCE_BETWEEN_PORTS * 1;//... see above, #2
	public static final double HEIGHT_3 = FIRST_HEIGHT + DISTANCE_BETWEEN_PORTS * 2;//...#3
	public static final double ACCEPTABLE_ERROR = 0.1;//inches
	public static final double SLOW_DOWN_DISTANCE = 5;//inches
	public static final double MIN_SPEED = 0.2;//(fraction/1)

	public double maxSpeed = 1;
	public int targetPort;
	public Spark clawSpark;

	public MoveClawToHeight(int target, Spark spark) {//constructor
		targetPort = target;
		clawSpark = spark;
		// Use requires() here to declare subsystem dependencies
		//requires(Robot.m_subsystem);
	}

	public double GetTargetClawHeight(){
		switch(targetPort){
			case 1:
				return HEIGHT_1;
			case 2:
				return HEIGHT_2;
			case 3:
				return HEIGHT_3;
		}
		System.out.println("GetTargetClawHeight failed, see script MoveClawToHeight");
		return 0;
	}

    //!\\ REPLACE THIS WITH ACTUAL METHOD //!\\
    //Should return the height of the claw (in inches) realative to the ground. This value should be greater than 0
    public double GetClawHeight(){
		System.out.println("!!!//////IMPLEMENT GetClawHeight\\\\\\!!!");
		return 0;//return distance of claw from floow in inches
    }

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double off = GetClawHeight() - GetTargetClawHeight();
		double absOff = off > 0 ? off: -off;
		double speed = absOff > SLOW_DOWN_DISTANCE ? 1 : absOff/SLOW_DOWN_DISTANCE;
		speed *= maxSpeed;
		speed = off > 0 ? speed : -speed;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		double off = GetClawHeight() - GetTargetClawHeight();
		off = off > 0 ? off: -off;
		return (off < ACCEPTABLE_ERROR);
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
