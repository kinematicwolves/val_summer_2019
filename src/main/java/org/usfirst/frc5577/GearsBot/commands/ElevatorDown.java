package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorDown extends Command {

	private double speed = 0;

	public ElevatorDown(double speed) {
		requires(Robot.lift);
		this.speed = speed;
	}

	@Override
	protected void initialize() {
		Robot.lift.moveLift(-speed);
	}

	@Override
	protected void execute() {

	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void interrupted() {
		Robot.lift.moveLift(0);
	}

}
