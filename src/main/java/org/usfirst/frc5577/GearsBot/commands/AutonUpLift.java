package org.usfirst.frc5577.GearsBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonUpLift extends CommandGroup {
	public AutonUpLift() {
		addSequential(new ElevatorUp(0.2, 0.3));
	}
}
