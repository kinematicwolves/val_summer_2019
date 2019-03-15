package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.*;

import edu.wpi.first.wpilibj.command.Command;

public class ManualElevatorControl extends Command {
  public ManualElevatorControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.enablePID();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double speed = -Robot.oi.manipulatorController.getRawAxis(Robot.oi.LEFT_AXIS_Y);

    speed = Math.signum(speed) * Math.pow(speed, 2);
    Robot.elevator.moveSetpoint(speed);
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
