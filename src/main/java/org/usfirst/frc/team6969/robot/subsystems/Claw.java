package org.usfirst.frc.team6969.robot.subsystems;

import org.usfirst.frc.team6969.robot.OI;
import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
import org.usfirst.frc.team6969.robot.commands.*;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Claw extends Subsystem {
	
	private double curSpeed = 0; //-1 to 1
	private static double maxCurrentIn = 10; // this is the current limit when the claw stops squeezing. With the redline, the claw burns at 110 Amps

	public static Spark clawMotor;
	
	private static double current;
	
    public void initDefaultCommand() {
    	setDefaultCommand(new MoveClaw());
    	current = 0.0;
    	clawMotor = RobotMap.clawMotor;
    }
    
    public void move(OI oi) {
    	if(clawMotor == null ) // prevents clawMotor from being null
    	{
    		this.initDefaultCommand();
    	}
    	
    	current = Robot.pdp.getCurrent(1);
    	if(current > maxCurrentIn)
    	{
    		//Robot.oi.getController().setRumble(RumbleType.kLeftRumble, 1);
    		
    	}
    	
    	if (oi.squareButton.get())
    		setCurSpeed(1.0);
    	else if (oi.circleButton.get())
    		setCurSpeed(-1.0);
    	else
    		setCurSpeed(0.0);	//not moving
    }
    
    public void stop() {
    	clawMotor.set(0);
    }
    
    public double getCurSpeed() {
		return curSpeed;
	}

	public void setCurSpeed(double curSpeed) {
		this.curSpeed = curSpeed;
		clawMotor.set(this.curSpeed);
	}
    
}

