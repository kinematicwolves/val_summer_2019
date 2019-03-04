package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.*;

import edu.wpi.first.wpilibj.command.Command;

public class ManualArmControl extends Command {
  public ManualArmControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.arm.enablePID();

    double speed = Robot.oi.manipulatorController.getRawAxis(Robot.oi.RIGHT_AXIS_Y);

    if (Robot.arm.isPIDenabled()) {
      Robot.arm.moveShoulderSetpoint(speed);
    } else {
      Robot.arm.move(speed);
      Robot.arm.setAngle(Robot.arm.getAngle());
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
    Robot.arm.stop();
  }
}
