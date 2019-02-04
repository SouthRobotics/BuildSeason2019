package org.usfirst.frc.team6969.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogPot extends AnalogInput {
    
    public double getAngle() {
        double degrees = pot.get();
        return degrees;
    }

}

