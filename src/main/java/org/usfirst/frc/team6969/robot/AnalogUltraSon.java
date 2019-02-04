/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */

public class AnalogUltraSon extends AnalogInput/*uses commands from analog class*/{    //custom class to convert analog input into inches
    
    public int channel;

    public AnalogUltraSon(int channel)
    {
        super(channel);//creates AnalogInput object
        this.channel = channel;
    }

    public double getInches()//custom converter method
    {
        double volt = 0;
        volt = super.getVoltage()*.5*.394;  //.5 is constant for volts to cm, .394 is cm to in
        return volt;
    }
    
}
