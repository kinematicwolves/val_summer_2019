// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5577.GearsBot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Run if robot is starting in the middle position.
 *	Gets Game Specific Message to determine location of switches and scale.
 */
public class AutonDriveFromCenter extends CommandGroup {
    
	private String gameData;
	
	public  AutonDriveFromCenter() {
    	
    	
    	// Get string of three characters
    	// Alliance Switch, Scale, and Opponents' Switch will be 0, 1, and 2 respectively
    	// Each character will be either L or R for Left or Right respectively
    	gameData = DriverStation.getInstance().getGameSpecificMessage();

    	if (gameData != null && gameData.length() > 0) {
	    	if (gameData.charAt(0) == 'L') { 
	    		addSequential(new DriveForward(5));
	    		addSequential(new Turn(-90));
	    		addSequential(new DriveForward(6));
	    		addSequential(new Turn(90));
//	    		addSequential(new UpLift(0.2, 4));
//				addSequential(new UpLift(0, 1));
	    		addSequential(new DriveForward(3));
//				addSequential(new PlaceCube());
//				addSequential(new ShiftClaw());
	        } else {
	        	addSequential(new DriveForward(5));
	            addSequential(new Turn(90));
	           	addSequential(new DriveForward(4));
	            addSequential(new Turn(-90));
//	            addSequential(new UpLift(0.2, 4));
//				addSequential(new UpLift(0, 1));
	            addSequential(new DriveForward(3));
//	            addSequential(new PlaceCube());
//				addSequential(new ShiftClaw());
	    	}
    	}
    	}  
}
