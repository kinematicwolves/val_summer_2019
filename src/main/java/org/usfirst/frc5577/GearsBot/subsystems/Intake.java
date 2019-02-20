package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {
	public Intake() {
		addChild(RobotMap.blackTalonSRX);
	}

	public void intakeBall(double speed) {
		RobotMap.blackTalonSRX.set(-speed);
	}

	public void shootOutBall(double speed) {
		RobotMap.blackTalonSRX.set(speed);
	}

	@Override
	protected void initDefaultCommand() {
	}

}
