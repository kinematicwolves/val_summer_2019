package org.usfirst.frc5577.GearsBot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
public class Auton extends CommandGroup {
    
	private String gameData;
	
	public  Auton(int i) {
    	
    	
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
	    		addSequential(new DriveForward(3));
	        } else {
	        	addSequential(new DriveForward(5));
	            addSequential(new Turn(90));
	           	addSequential(new DriveForward(4));
	            addSequential(new Turn(-90));
	            addSequential(new DriveForward(3));
	    	}
    	}
    	}  public class AutonDriveFromLeft extends CommandGroup {
            private String gameData;
            public AutonDriveFromLeft() {
                // Get string of three characters
                // Alliance Switch, Scale, and Opponents' Switch will be 0, 1, and 2 respectively
                // Each character will be either L or R for Left or Right respectively
                gameData = DriverStation.getInstance().getGameSpecificMessage();
                if (gameData != null && gameData.length() > 0) {
                    if (gameData.charAt(0) == 'L'){ // The switch is on the left side
                        addSequential(new DriveForward(3)); // Increase this!
                        addSequential(new Turn(90));
                        addSequential(new DriveForward(1));
                    } 
                    else {
                        addSequential(new DriveForward(9));
                    }
                }
            }
        }
        /**
 *	Run if robot is starting in the right position.
 *	Gets Game Specific Message to determine location of switches and scale.
 */
public class AutonDriveFromRight extends CommandGroup {
	private String gameData;
	public AutonDriveFromRight() {
		// Get string of three characters
    	// Alliance Switch, Scale, and Opponents' Switch will be 0, 1, and 2 respectively
    	// Each character will be either L or R for Left or Right respectively
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData != null && gameData.length() > 0) {
			if (gameData.charAt(0) == 'R') { // The switch is on the right
				addSequential(new DriveForward(3)); // Increase this!
				addSequential(new Turn(-90));
				addSequential(new DriveForward(1));
				} 
			else {
				addSequential(new DriveForward(10));
			}
		}
	}
}
public class AutonDriveStraight extends CommandGroup {
    public  AutonDriveStraight() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
        addSequential(new DriveForward(10));
    }  
    
}
public class AutonTurn extends CommandGroup {
	
	public AutonTurn() {
		addSequential(new Turn(90));
	}

}
public class AutonUpLift extends CommandGroup {
	public AutonUpLift() {
		addSequential(new ElevatorUp(0.2, 0.3));
	}
}

}

