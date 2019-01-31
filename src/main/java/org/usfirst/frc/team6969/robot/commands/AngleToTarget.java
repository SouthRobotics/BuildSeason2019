package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AngleToTarget extends CommandGroup {
    public AngleToTarget() {
        addSequential(new RotateToPixyTarget(RotateToPixyTarget.cen));
        
    }
    
}
