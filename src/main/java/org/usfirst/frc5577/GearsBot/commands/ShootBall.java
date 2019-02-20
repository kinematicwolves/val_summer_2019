package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShootBall extends Command {

	private double speed = 0.7;
	private double time = 0;

	public ShootBall() {
		this(0.8);
	}

	public ShootBall(double speed) {
		requires(Robot.intake);
		this.speed = speed;
	}

	public ShootBall(double speed, double time) {
		this(speed);
		this.time = time;
	}

	@Override
	protected void initialize() {
		Robot.intake.shootOutBall(speed);
	}

	@Override
	protected void execute() {

	}

	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	@Override
	protected void interrupted() {
		Robot.intake.shootOutBall(0);
	}

}
