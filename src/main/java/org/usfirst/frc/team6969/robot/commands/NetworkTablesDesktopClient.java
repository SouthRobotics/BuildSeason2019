package org.usfirst.frc.team6969.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
public class NetworkTablesDesktopClient extends Command {
	public NetworkTablesDesktopClient() {
		// Use requires() here to declare subsystem dependencies
		//requires(Robot.m_subsystem);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable Ball = inst.getTable("GRIP/BallReport");
    NetworkTable Hatch = inst.getTable("GRIP/HatchReport");
    NetworkTableEntry xEntry = Ball.getEntry("centerX");
    NetworkTableEntry yEntry = Hatch.getEntry("centerX");
    inst.startClientTeam(6969);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
   while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      double x = xEntry.getDouble(-1);
      double y = yEntry.getDouble(-1);
      System.out.println("X Ball: " + x + " X Hatch: " + y);
    }
  }
	

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
