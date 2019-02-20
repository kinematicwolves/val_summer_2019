package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class BackUp extends Command {
	double speed = 0.75;
    double time = 1.5;
	
	public  BackUp() {
      requires(Robot.driveTrain);		 
      }
  
  public BackUp(double speed) {
  	this();
  	this.speed = speed;
  }
  
  public BackUp(double speed, double time) {
  	this(speed);
  	this.time = time;
  }
  
  protected void initialize() {
	  setTimeout(time); //Or change back to 5 if something goes wrong
	  }

  protected void execute() {
  	Robot.driveTrain.driveTrainBackward(speed);
  }

    protected boolean isFinished() {
      return isTimedOut(); 
  }

    protected void end() {  	
    	Robot.driveTrain.stop(); 
    	}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  	end();
  }
}


