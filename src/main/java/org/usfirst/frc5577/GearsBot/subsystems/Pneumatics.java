package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Pneumatics extends Subsystem {

	private boolean isHighGear = false;
	private boolean isWristUp = false;
	private boolean isHatchPanelOut = false;
	private boolean isClimbExtended = false;

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}

	public void shiftGear() {
		if (isHighGear) {
			shiftToLowGear();
		} else {
			shiftToHighGear();
		}
	}

	public void shiftWrist() {
		if (isWristUp) {
			downWrist();
		} else {
			upWrist();
		}
	}

	public void shiftHatchPanel() {
		if (isHatchPanelOut) {
			pullPanelIn();
		} else {
			pushPanelOut();
		}
	}

	public void extendClimb() {
		if (isClimbExtended) {
			retractClimbDown();
		} else {
			extendClimbUp();
		}
	}

	public boolean isHighGear() {
		return isHighGear;
	}

	public boolean isWristUp() {
		return isWristUp;
	}

	public boolean isHatchPanelOut() {
		return isHatchPanelOut;
	}

	public boolean isClimbExtended() {
		return isClimbExtended;
	}

	private void shiftToHighGear() {
		RobotMap.driveTrainSwitch.set(Value.kForward);
		isHighGear = true;
	}

	private void shiftToLowGear() {
		RobotMap.driveTrainSwitch.set(Value.kReverse);
		isHighGear = false;
	}

	private void upWrist() {
		RobotMap.wristSwitch.set(Value.kForward);
		isWristUp = true;

	}

	private void downWrist() {
		RobotMap.wristSwitch.set(Value.kReverse);
		isWristUp = false;
	}

	private void pushPanelOut() {
		RobotMap.hatchPanelSwitch.set(Value.kForward);
		isHatchPanelOut = true;
	}

	private void pullPanelIn() {
		RobotMap.hatchPanelSwitch.set(Value.kReverse);
		isHatchPanelOut = false;
	}

	private void extendClimbUp() {
		RobotMap.climbSwitch.set(Value.kForward);
		isClimbExtended = true;
	}

	private void retractClimbDown() {
		RobotMap.climbSwitch.set(Value.kReverse);
		isClimbExtended = false;
	}

	public void stop() {
		RobotMap.driveTrainSwitch.set(Value.kOff);
	}

}
