// package org.usfirst.frc5577.GearsBot.commands;

// import org.usfirst.frc5577.GearsBot.Robot;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class ShiftClimb extends Command {

// private boolean initialClimbState;
// private double time = 0.5;

// public ShiftClimb() {
// requires(Robot.pneumatics);

// }

// protected void initialize() {
// setTimeout(time);
// initialClimbState = Robot.pneumatics.isClimbExtended();
// }

// @Override
// protected boolean isFinished() {
// return isTimedOut();
// }

// // Called once after isFinished returns true
// protected void end() {
// // Robot.pneumatics.stop();
// }

// protected void execute() {
// SmartDashboard.putBoolean("initial climb state ", initialClimbState);
// if (initialClimbState != Robot.pneumatics.isClimbExtended()) {
// Robot.pneumatics.extendClimb();
// }
// }

// }
