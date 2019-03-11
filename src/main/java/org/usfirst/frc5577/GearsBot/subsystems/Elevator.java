package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

	private StringBuilder sb = new StringBuilder();
	int loops = 0;

	public Elevator() {
		// RobotMap.clearTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
		// 0, 0);
		// RobotMap.clearTalonSRX.configNominalOutputForward(+0.0, -0);
		// RobotMap.clearTalonSRX.configPeakOutputForward(+12.0, -12);
		// RobotMap.clearTalonSRX.selectProfileSlot(0, 0);
		// RobotMap.clearTalonSRX.config_kF(0, 0.01, 0);
		// RobotMap.clearTalonSRX.config_kP(0, 0.0, 0);
		// RobotMap.clearTalonSRX.config_kI(0, 0.0, 0);
		// RobotMap.clearTalonSRX.config_kD(0, 0.0, 0);
		// RobotMap.clearTalonSRX.set(ControlMode.PercentOutput, 0);
	}

	public void moveLift(double speed) {
		// double motorOutput = RobotMap.clearTalonSRX.getMotorOutputVoltage() /
		// RobotMap.clearTalonSRX.getBusVoltage();
		// sb.append("\tout:");
		// sb.append(motorOutput);
		// sb.append("\tspeed");
		// sb.append(RobotMap.clearTalonSRX.getSelectedSensorVelocity(0));

		// double targetSpeed = speed; // 6000 RPM target
		RobotMap.elevatorTalonSRX.set(-speed);

		// sb.append("/terr");
		// sb.append(RobotMap.clearTalonSRX.getClosedLoopError(0));
		// sb.append("/ttrg:");
		// sb.append(targetSpeed);

		// sb.setLength(0);

	}

	public void stop() {
		RobotMap.elevatorTalonSRX.set(ControlMode.PercentOutput, 0);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
