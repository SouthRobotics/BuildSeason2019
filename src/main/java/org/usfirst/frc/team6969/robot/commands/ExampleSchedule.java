package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ExampleSchedule extends CommandGroup {
    public ExampleSchedule() {
        addSequential(new moveBotFast());
        addSequential(new moveBotSlow());
        addSequential(new moveBotFast());
    }
    
}
