/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team6969.robot.commands.GripPipelineHATCH;
import org.usfirst.frc.team6969.robot.commands.GripPipelineBall;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//OI class connects sets up controller values. Basically a map of controller buttons.

	//Below are examples from FIRST:
	
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	
	
	//XBOX 360 controller mappings (we got these from DriverStation)
	/*
	 * Buttons: (gives pressed or not pressed)
	 * 1: X
	 * 2: A
	 * 3: B
	 * 4: Y
	 * 5: Left Bumper
	 * 6: Right Bumper
	 * 7: Back Left Trigger
	 * 8: Back Right Trigger
	 * 9: Back
	 * 10: Start
	 * 
	 * Axes: (gives degrees or amount pressed)
	 * 1. 
	 * 
	 */
	
	//XBOX 1 controller mappings (we got these from DriverStation)
		/*
		 * Buttons: (gives pressed or not pressed)
		 * 1: A
		 * 2: B
		 * 3: X
		 * 4: Y
		 * 5: Left Bumper
		 * 6: Right Bumper
		 * 7: View Button
		 * 8: Menu Button
		 * 9: Left Joystick
		 * 10: Right Joystick
		 * 
		 * Axes: (gives degrees or amount pressed)
		 * 0. Left Joystick X Axis
		 * 1. Left Joystick Y Axis 
		 * 2. Back Left Trigger
		 * 3. Back Right Trigger
		 * 4. Right Joystick X Axis
		 * 5. Right Joystick Y Axis
		 * 
		 */
	
	//PS4 controller mappings (we got these from DriverStation)
		/*
		 * Buttons:	(gives pressed or not pressed)
		 * 1: Square
		 * 2: X
		 * 3: Circle
		 * 4: Triangle
		 * 5: Left Bumper
		 * 6: Right Bumper
		 * 7: Back Left Trigger
		 * 8: Back Right Trigger
		 * 9: Share
		 * 10: Options
		 * 11: Power
		 * 12: Touchpad
		 * 
		 * Axes:	(gives degrees or amount pressed)
		 * 0. Left Joystick X Axis
		 * 1. Left Joystick Y Axis
		 * 2. Right Joystick X Axis
		 * 3. Back Left Trigger
		 * 4. Back Right Trigger
		 * 5. Right Joystick Y Axis
		 * 
		 */
	
	//For ALL XBOX/PS4 controllers: dpad is a number (called POV) between 0 and 360, inclusive. Get number with controller.getPOV()
	
	
	private XboxController controller = new XboxController(0); // Must be USB port 1 in DriverStation.

	//PS4 controller running DS4 for windows
	public Button squareButton = new JoystickButton(controller, 1),
			xButton = new JoystickButton(controller, 2),
			circleButton = new JoystickButton(controller, 3),
			triangleButton = new JoystickButton(controller, 4),
			leftBumper = new JoystickButton(controller,5),
			rightBumper = new JoystickButton(controller, 6),
			backLeftTrigger = new JoystickButton(controller, 7),
			backRightTribber = new JoystickButton(controller, 8),
			shareButton = new JoystickButton(controller, 9),
			optionsButton = new JoystickButton(controller, 10),
			powerButton = new JoystickButton(controller, 11),
			touchpadButton = new JoystickButton(controller, 12);
	
	public int leftYAxis = 1;
	public int rightYAxis = 5;
    
    public OI() {
		circleButton.whenPressed(new GripPipelineHATCH());
		triangleButton.whenPressed(new GripPipelineBall());
	}
    
    public XboxController getController() {
        return controller;
    }
    
        
}