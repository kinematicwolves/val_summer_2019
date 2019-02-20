package org.usfirst.frc5577.GearsBot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonDriveFromLeft extends CommandGroup {
	
	private String gameData;
	
	public AutonDriveFromLeft() {
		
		// Get string of three characters
    	// Alliance Switch, Scale, and Opponents' Switch will be 0, 1, and 2 respectively
    	// Each character will be either L or R for Left or Right respectively
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData != null && gameData.length() > 0) {
			if (gameData.charAt(0) == 'L'){ // The switch is on the left side
				addSequential(new DriveForward(3)); // Increase this!
//				addSequential(new UpLift(0.2, 4));
//				addSequential(new UpLift(0, 1));
				addSequential(new Turn(90));
				addSequential(new DriveForward(1));
//				addSequential(new PlaceCube());
//				addSequential(new ShiftClaw());
//				addSequential(new ShiftClaw());
			} 
			//else if (gameData.charAt(1) == 'L') { // The scale is on the left side
//				addSequential(new DriveForward(15));
//				addSequential(new Turn(-90));
//				addSequential(new DriveForward(.25));
//				addSequential(new UpLift(0.3, 4));
//				addSequential(new UpLift(0, 1));
//				addSequential(new PlaceCube());
//				addSequential(new ShiftClaw());
//				addSequential(new ShiftClaw());
//			} 
			else {
				addSequential(new DriveForward(9));
			}
		}
	}
}
