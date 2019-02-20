package org.usfirst.frc5577.GearsBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonTurn extends CommandGroup {
	
	public AutonTurn() {
		addSequential(new Turn(90));
	}

}
