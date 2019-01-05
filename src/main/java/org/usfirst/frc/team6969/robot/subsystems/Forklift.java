package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.OI;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.*;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Forklift extends Subsystem {
	public static Spark forkliftMotor;
	
	private double curSpeed;
	
    public void initDefaultCommand() {
    	setDefaultCommand(new MoveForklift());
    	forkliftMotor = RobotMap.forkliftMotor;
    	setCurSpeed(0.0);
    }
    
    public void move(OI oi) {
    	if(forkliftMotor == null )
    	{
    		this.initDefaultCommand();
    	}
    	
    	if (oi.getController().getPOV() == 0)
    		setCurSpeed(1.0);
    	else if (oi.getController().getPOV() == 180)
    		setCurSpeed(-1.0);
    	else
    		setCurSpeed(0.0);	//when let go of dpad
    }
    
    public void stop() {
    	forkliftMotor.set(0);
    }

	public double getCurSpeed() {
		return curSpeed;
	}

	public void setCurSpeed(double curSpeed) {
		this.curSpeed = curSpeed;
		forkliftMotor.set(this.curSpeed);
	}
    
}

