package org.usfirst.frc5577.GearsBot.subsystems.pid;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceTalon extends WPI_TalonSRX implements PIDSource {
    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return this.getSensorCollection().getQuadraturePosition();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // do nothing
    }

    public PIDSourceTalon(int CANid) {
        super(CANid);
        this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        this.getSensorCollection().setQuadraturePosition(0, 10);
    }

}