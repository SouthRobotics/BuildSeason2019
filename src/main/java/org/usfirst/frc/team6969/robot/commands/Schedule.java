package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Schedule extends CommandGroup{
    public Schedule() {
        addSequential(new moveBotFast());
        addSequential(new moveBotSlow());
        addSequential(new moveBotFast());
    }
    
}
