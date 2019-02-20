package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.*;

public class Arm extends Subsystem {

    // private static final int TIMEOUT = 30;
    // private int holdPosition;

    public Arm() {
        // RobotMap.redTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        // 0, TIMEOUT);
        // RobotMap.redTalonSRX.configNominalOutputForward(+0.0, TIMEOUT);
        // RobotMap.redTalonSRX.configNominalOutputReverse(-0.0, TIMEOUT);
        // RobotMap.redTalonSRX.configPeakOutputForward(+1.0, TIMEOUT);
        // RobotMap.redTalonSRX.configPeakOutputReverse(-1.0, TIMEOUT);
        // RobotMap.redTalonSRX.selectProfileSlot(0, 0);
        // RobotMap.redTalonSRX.config_kF(0, 0.0, TIMEOUT);
        // RobotMap.redTalonSRX.config_kP(0, 0.15, TIMEOUT);
        // RobotMap.redTalonSRX.config_kI(0, 0.0, TIMEOUT);
        // RobotMap.redTalonSRX.config_kD(0, 0.01, TIMEOUT);
    }

    public void liftArm(double speed) {
        RobotMap.redTalonSRX.set(speed);
    }

    public void lowerArm(double speed) {
        RobotMap.redTalonSRX.set(-speed);
    }

    // public void hold() {
    // RobotMap.redTalonSRX.set(ControlMode.Position, holdPosition);
    // }

    public void stop() {
        RobotMap.redTalonSRX.set(0);
    }

    public void setSensorPosition() {
        // /**
        // * Grab the 360 degree position of the MagEncoder's absolute position, and
        // * intitally set the relative sensor to match.
        // */
        // holdPosition =
        // RobotMap.redTalonSRX.getSensorCollection().getPulseWidthPosition();

        // /* Set the quadrature (relative) sensor to match absolute */
        // RobotMap.redTalonSRX.setSelectedSensorPosition(holdPosition, 0, TIMEOUT);
    }

    @Override
    protected void initDefaultCommand() {
    }

}
