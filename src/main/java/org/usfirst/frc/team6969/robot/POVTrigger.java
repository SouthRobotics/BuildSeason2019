package org.usfirst.frc.team6969.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;


        public class POVTrigger extends Trigger {
                Joystick leftStick = OI.leftJoy;
                Joystick rightStick = OI.rightJoy;

                public static final int rightX = 1;
                public static final int upRightX = 1;
                public static final int upX = 0;
                public static final int upLeftX = -1;
                public static final int leftX = -1;
                public static final int downLeftX = -1;
                public static final int downX = 0;
                public static final int downRightX = 1;

                public static final int rightY = 0;
                public static final int upRightY = 1;
                public static final int upY = 1;
                public static final int upLeftY = -1;
                public static final int leftY = 0;
                public static final int downLeftY = -1;
                public static final int downY = -1;
                public static final int downRightY = -1;

                public POVTrigger(int x, int y){
                }

                public boolean get() {
                        if (true)
                                return true;
                        else
                                return false;
                }

                public POVHat(Joystick stick, int x, int y){
                }

        
                POVTrigger lRight = new POVTrigger(POVTrigger.rightX, POVTrigger.rightY),
		lUpRight = new POVTrigger(POVTrigger.upRightX, POVTrigger.upRightY),
		lUp = new POVTrigger(POVTrigger.upX, POVTrigger.upY),
		lUpLeft = new POVTrigger(POVTrigger.upLeftX, POVTrigger.upLeftY),
		lLeft = new POVTrigger(POVTrigger.leftX, POVTrigger.leftY),
		lDownLeft = new POVTrigger(POVTrigger.downLeftX, POVTrigger.downLeftY),
		lDown = new POVTrigger(POVTrigger.downX, POVTrigger.downY),
		lDownRight = new POVTrigger(POVTrigger.downRightY, POVTrigger.downRightY);

		POVTrigger rRight = new POVTrigger(POVTrigger.rightX, POVTrigger.rightY),
		rUpRight = new POVTrigger(POVTrigger.upRightX, POVTrigger.upRightY),
		rUp = new POVTrigger(POVTrigger.upX, POVTrigger.upY),
		rUpLeft = new POVTrigger(POVTrigger.upLeftX, POVTrigger.upLeftY),
		rLeft = new POVTrigger(POVTrigger.leftX, POVTrigger.leftY),
		rDownLeft = new POVTrigger(POVTrigger.downLeftX, POVTrigger.downLeftY),
		rDown = new POVTrigger(POVTrigger.downX, POVTrigger.downY),
		rDownRight = new POVTrigger(POVTrigger.downRightY, POVTrigger.downRightY);
                

        }



                

