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

                public POVTrigger(Joystick stick, int x, int y) {
                

        }



}

                

