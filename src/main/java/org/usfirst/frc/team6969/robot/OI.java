/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

//import org.usfirst.frc.team6969.robot.commands.ExampleSchedule;
import org.usfirst.frc.team6969.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//OI class connects sets up controller values. Basically a map of controller buttons.

	//Below are examples from FIRST:
	
	/* CREATING BUTTONS
	One type of button is a joystick button which is any button on a joystick.
	You create one by telling it which joystick it's on and which button number it is.
	Joystick stick = new Joystick(port);
	Button button = new JoystickButton(stick, buttonNumber);

	There are a few additional built in buttons you can use. Additionally,
	by subclassing Button you can create custom triggers and bind those to
	commands the same as any other Button.

	// TRIGGERING COMMANDS WITH BUTTONS
	Once you have a button, it's trivial to bind it to a button in one of three ways:
	  Start the command when the button is pressed and let it run the command
	  until it is finished as determined by it's isFinished method.
	  button.whenPressed(new ExampleCommand());

	  Run the command while the button is being held down and interrupt it once
	  the button is released.
	  button.whileHeld(new ExampleCommand());

	  Start the command when the button is released and let it run the command
	  until it is finished as determined by it's isFinished method.
	  button.whenReleased(new ExampleCommand());
	*/

	public int yAxis = 1;

	public Joystick leftJoy = new Joystick(0);
	public Button lButton1 = new JoystickButton(leftJoy, 1),
		lButton2 = new JoystickButton(leftJoy, 2),
		lButton3 = new JoystickButton(leftJoy, 3),
		lButton4 = new JoystickButton(leftJoy, 4),
		lButton5 = new JoystickButton(leftJoy, 5),
		lButton6 = new JoystickButton(leftJoy, 6),
		lButton7 = new JoystickButton(leftJoy, 7),
		lButton8 = new JoystickButton(leftJoy, 8),
		lButton9 = new JoystickButton(leftJoy, 9),
		lButton10 = new JoystickButton(leftJoy, 10),
		lButton11 = new JoystickButton(leftJoy, 11),
		lButton12 = new JoystickButton(leftJoy, 12);

	public Joystick rightJoy = new Joystick(1);
	public Button rButton1 = new JoystickButton(rightJoy, 1),
		rButton2 = new JoystickButton(rightJoy, 2),
		rButton3 = new JoystickButton(rightJoy, 3),
		rButton4 = new JoystickButton(rightJoy, 4),
		rButton5 = new JoystickButton(rightJoy, 5),
		rButton6 = new JoystickButton(rightJoy, 6),
		rButton7 = new JoystickButton(rightJoy, 7),
		rButton8 = new JoystickButton(rightJoy, 8),
		rButton9 = new JoystickButton(rightJoy, 9),
		rButton10 = new JoystickButton(rightJoy, 10),
		rButton11 = new JoystickButton(rightJoy, 11),
		rButton12 = new JoystickButton(rightJoy, 12);
    
    public OI() {
		/*rButton2.whenPressed(new ManualOverride());
		lButton3.whenPressed(new RotateToPixyTarget(Robot.pixyCenter));
		lButton4.whenPressed(new RotateChassisToAngle(-90));
		lButton5.whenPressed(new RotateChassisToAnglePID(-90));
		rButton5.whileHeld(new SpinRollerIntake(true));
		rButton6.whileHeld(new SpinRollerIntake(false));
		rButton7.whileHeld(new SpinBottomJoint(0.25));
		rButton8.whileHeld(new SpinMiddleJoint(0.25));
		rButton9.whileHeld(new SpinTopJoint(0.25));*/
		rButton10.whileHeld(new TestMotor());

	}
	
    /*public XboxController getController() {
        return controller;
    }*/
    
        
}