/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TurnUntilLinedUp extends Turn {

  double x;
  double y;
  double area;

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  public TurnUntilLinedUp() {
    super();
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    x = tx.getDouble(0.0);
    if (x < 0) {
      turnValue = turnValue * -1;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    super.execute();
    // read values periodically
    x = tx.getDouble(0.0);
    System.out.println("The value of x from Limelight is " + x);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    Robot.driveTrain.turn(turnValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (x > -2 || x < 2) {
      return true;
    }
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
