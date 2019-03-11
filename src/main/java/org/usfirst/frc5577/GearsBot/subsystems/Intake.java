package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {
	public Intake() {
		addChild(RobotMap.intakeTalonSRX);
	}

	public void intakeBall(double speed) {
		RobotMap.intakeTalonSRX.set(-speed);
	}

	public void shootOutBall(double speed) {
		RobotMap.intakeTalonSRX.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
	}

}
