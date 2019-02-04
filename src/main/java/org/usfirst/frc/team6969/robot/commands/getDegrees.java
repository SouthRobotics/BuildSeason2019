package org.usfirst.frc.team6969.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team6969.robot.RobotMap;\

public class getDegrees {
    public getDegrees{
        public static double potDegrees = (RobotMap.pot.get() / 5);
        return potDegrees;
    }
}