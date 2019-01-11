/**written by Rishi Astra. If you have questions about the code or the code makes no sense ask me**/
/**
 * Best way to test this script would be to disable all other movement.
 * Expected calls to methods:
 * Initialize when robot turns on
 * Execute when robot is on and not finished
 * 
 * ||||If i misunderstood when and which methods are called (above is wrong), this script won't work!||||

 * 
 * isFinished will return true if robot is within angleErrorThreshold if and only if stopWhenHitTargetAngle = true
 * 
 **/

package org.usfirst.frc.team6969.robot.commands;

import org.usfirst.frc.team6969.robot.Robot;
import org.usfirst.frc.team6969.robot.RobotMap;
//import org.usfirst.frc.team6969.robot.subsystems.TestMovement;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//All imports below are the default imports that come with the FRC package
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class TestMovementCommand extends Command {

	private static final boolean invertRight = false;// use these if one side of robot is moving backwards
	private static final boolean invertLeft = false;//see above
	private static final boolean stopWhenHitTargetAngle = false;//if true, isFinished returns true when the robot reaches the target angle.
	private static final boolean keepCommandWhenFinished = true;//keep trying to reach target angle even if you reached it
	//if (absolute value) |targetAngle - angle| < angleErrorThreshold, robot will stop turning (finichedMoving = true)
	private static final double angleErrorThreshold = 1;
	private static final double movementErrorThreshold = 1;//!\ no units specified yet, when movement is implemented add units.

	//if (absolute value) |targetAngle - angle| < maxSpeedAtWhatError, the speed is multiplied by|targetAngle - angle| < maxSpeedAtWhatError
	private static final double maxSpeedAtWhatError = 45; 
	private static final double maxSpeedAtWhatDist = 2;//for movement (not turning, moving forward/backward)
	//a multiplier for speed. Probably should be within the range [0, 1]
	private static final double turnPower = 1;

	public static double targetAngle = 90;// change to rotate robot. (the robot tries to reach this angle, relative to gyro angle)
	public static boolean finishedMoving = false;// is the robot currently turning? (see angleErrotThreshold)
	private static AnalogGyro gyro;// reference needed to sense rotation
	private static DifferentialDrive drive;// reference needed to drive

	//these variables will be used when the robot automatically moves forward a target distance. Not in use now. No units specified
	private double targetDist;
	private double coveredDist;
	
	//need to make constructor
	
//	public TestMovement sub;//a reference to TestMovement, just incase it will come in handy later
	
	public TestMovementCommand() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		
    	this.initialize(); // after the TeleopDrive object is declared
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//		sub = Robot.testMovement;
		gyro = RobotMap.gyro;
		gyro.reset();
		gyro.calibrate();//unsure if needed, might reset gyro angle
		drive = RobotMap.drive;
	}
	//not in use, use this for autonomous movement in future.
	public void SetTargetDistance(double dist) {
		targetDist = dist;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
//		if(Robot.m_oi.xButton.get()) {
		turn(0.0);
//		}else {
////			turn(-90);
//		}
		//use this for autonomous movement forward/backward
		/**
		if(Math.abs(targetDist-coveredDist) > movementErrorThreshold) {
			double mult = targetDist-coveredDist;
			mult = mult > 0 ? 1: -1;
			mult = mult > maxSpeedAtWhatDist ? 1 : mult / maxSpeedAtWhatDist;
			move(mult);
		}
		**/
		// change 0 to speed of robot (-1, 1) if robot is moving forwards/backwards
		// during rotation

	}
	
	private double findShortestAngleBetween(double target, double value) {
		double temp = target - value;//get angle between
		//find shortest angle between (e.g., if target is 0 and value is 360, make sure it returns 0, not -360)
		temp += (temp > 180) ? -360 : (temp < -180) ? 360 : 0;
		return temp;
	}
	
	public void move(double power) {
		RobotMap.drive.tankDrive(power, power);
	}

	// power -- the desired speed of the robot (forward-backward) before turning.
	// Should be 0 unless robot is turning while moving.
	// gyro & drive -- references
	public void turn(double power) {
		//calculate the relative angle, the angle given by gyro compared to target angle
		//radians to degrees = 57.2958, that is why i multiply
		double readAngle = (gyro.getAngle())%360;//don't allow angle to exceed 360 by dividing by 360 and using remainder
		double error = findShortestAngleBetween(targetAngle, readAngle);
		//check if robot has reached target position (if so, set finishedMoving = true)
		if (error > -angleErrorThreshold && error < angleErrorThreshold) {
			finishedMoving = true;
			return;
			//if robot is not at target rotation, check if finishedMoving needs to be set to false (only if keepCommandWhenFinished = true)
		} else if (keepCommandWhenFinished) {
			finishedMoving = false;
		}

		if (finishedMoving)
			return;//if finishedMoving, you don't need to execute any movement code
		//set left and right speeds to power. Useful if robot needs to move while correcting its rotation
		double leftSpeed = power;
		double rightSpeed = power;

		//turn the robot (case 1, robot needs to turn clockwise)
		if (error > 0) {
			double absError = error;

			// Turn left by setting the left track to high speed and right to negative
			// speed. The speed is the max speed, unless the error is smaller than
			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError
			leftSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
			rightSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
		}
		//turn the robot (case 1, robot needs to turn counter-clockwise)
		if (error < 0) {
			double absError = error * -1;
			// Turn right by setting the right track to high speed and left to negative
			// speed. The speed is the max speed, unless the error is smaller than
			// maxSpeedAtWhatError, then the speed is error/maxSpeedAtWhatError

			leftSpeed -= absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
			rightSpeed += absError > maxSpeedAtWhatError ? turnPower : turnPower * absError / maxSpeedAtWhatError;
		}

		rightSpeed = invertRight ? -rightSpeed : rightSpeed;
		leftSpeed = invertLeft ? -leftSpeed : leftSpeed;

		// clamp
		if (rightSpeed > 1)
			rightSpeed = 1;
		if (rightSpeed < -1)
			rightSpeed = -1;
		if (leftSpeed > 1)
			leftSpeed = 1;
		if (leftSpeed < -1)
			leftSpeed = -1;

		// drive the robot using the values calculated.
		// either this or the drive.tankDrive below can be used.

		// set motor speeds
		//		    RobotMap.driveTrainLeftFront.set(leftSpeed);
		//		    RobotMap.driveTrainLeftBack.set(leftSpeed);
		//		    
		//		    RobotMap.driveTrainRightFront.set(rightSpeed);
		//		    RobotMap.driveTrainRightBack.set(rightSpeed);

		drive.tankDrive(leftSpeed, rightSpeed);// drive the robot using the values calculated

	}

	@Override
	protected boolean isFinished() {
		//Auto-generated method stub
		return finishedMoving && stopWhenHitTargetAngle;
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
