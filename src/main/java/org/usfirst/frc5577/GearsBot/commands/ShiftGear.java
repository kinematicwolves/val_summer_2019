package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShiftGear extends Command {
	
	private boolean initialGearState;
	private double time = 0.5;
	
	public ShiftGear() {
		requires(Robot.pneumatics);
		
	}
	protected void initialize() {
		setTimeout(time);
	  	initialGearState = Robot.pneumatics.isHighGear(); 
	  }

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}
	 // Called once after isFinished returns true
	  protected void end() {	  	
//	  	Robot.pneumatics.stop();
	  	}
	  
	protected void execute() {
		if (initialGearState == Robot.pneumatics.isHighGear()) {
			Robot.pneumatics.shiftGear();
		}
	}
}
