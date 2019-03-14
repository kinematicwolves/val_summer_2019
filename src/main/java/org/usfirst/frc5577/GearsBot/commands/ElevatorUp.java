package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;
import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorUp extends Command {

	private double speed = 0;
	private double time = 3;
	private double distanceInInches = 12;

	public ElevatorUp() {
		requires(Robot.elevator);
	}

	// public ElevatorUp(double distanceInFeet) {
	// this();
	// this.distanceInInches = 12 * distanceInFeet;
	// }

	public ElevatorUp(double speed) {
		this();
		this.speed = speed;
	}

	public ElevatorUp(double speed, double time) {
		this(speed);
		this.time = time;
	}

	@Override
	protected void initialize() {
		Robot.elevator.moveLift(speed);
	}

	@Override
	protected void execute() {

	}

	// @Override
	protected boolean isFinished() {
		// return isTimedOut();
		// System.out.println("Elevator encoder count: " +
		// RobotMap.elevatorEncoder.get());
		// System.out.println("Elevator encoder distance traveled: " +
		// RobotMap.elevatorEncoder.getDistance() / 12);

		// if (RobotMap.elevatorEncoder.getDistance() >= this.distanceInInches) {
		// System.out.println("Finished the command!");
		// return true;
		// }
		return false;
	}

	@Override
	protected void interrupted() {
		Robot.elevator.moveLift(0);
	}

}
