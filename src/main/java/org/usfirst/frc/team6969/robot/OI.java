/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import org.usfirst.frc.team6969.robot.commands.ManualOverride;
import org.usfirst.frc.team6969.robot.commands.RotateToAngle;
import org.usfirst.frc.team6969.robot.commands.RotateToPixyTarget;
import org.usfirst.frc.team6969.robot.commands.Schedule;
import org.usfirst.frc.team6969.robot.commands.moveBotFast;

import edu.wpi.first.wpilibj.XboxController;
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
	
	//XBOX 360 controller mappings (we got these from DriverStation)
	
	 Buttons: (gives pressed or not pressed)
	  1: X
	  2: A
	  3: B
	  4: Y
	  5: Left Bumper
	  6: Right Bumper
	  7: Back Left Trigger
	  8: Back Right Trigger
	  9: Back
	  10: Start
	  
	  Axes: (gives degrees or amount pressed)
	  1. 
	  	 
	//XBOX 1 controller mappings (we got these from DriverStation)
		 Buttons: (gives pressed or not pressed)
		  1: A
		  2: B
		  3: X
		  4: Y
		  5: Left Bumper
		  6: Right Bumper
		  7: View Button
		  8: Menu Button
		  9: Left Joystick
		  10: Right Joystick
		  
		  Axes: (gives degrees or amount pressed)
		  0. Left Joystick X Axis
		  1. Left Joystick Y Axis 
		  2. Back Left Trigger
		  3. Back Right Trigger
		  4. Right Joystick X Axis
		  5. Right Joystick Y Axis
		*/	
	
	private XboxController controller = new XboxController(1); // Must be USB port 1 in DriverStation.

	public Button aButton = new JoystickButton(controller, 1),
			bButton = new JoystickButton(controller, 2),
			xButton = new JoystickButton(controller, 3),
			yButton = new JoystickButton(controller, 4),
			leftBumper = new JoystickButton(controller,5),
			rightBumper = new JoystickButton(controller, 6),
			viewButton = new JoystickButton(controller, 7),
			menuButton = new JoystickButton(controller, 8),
			leftStick = new JoystickButton(controller, 9),	//push stick in not tilt
			rightStick = new JoystickButton(controller, 10);
	
	public int leftYAxis = 1;
	public int rightYAxis = 5;
    
    public OI() {
		viewButton.whenPressed(new ManualOverride());
		xButton.whenPressed(new RotateToPixyTarget(Robot.pixyCenter));
		aButton.whenPressed(new RotateToAngle(90));
		yButton.whenPressed(new Schedule());
		bButton.whenPressed(new moveBotFast());
	}
    
    public XboxController getController() {
        return controller;
    }
    
        
}