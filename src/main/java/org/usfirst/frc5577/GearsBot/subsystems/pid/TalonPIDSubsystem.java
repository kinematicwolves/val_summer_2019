package org.usfirst.frc5577.GearsBot.subsystems.pid;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class TalonPIDSubsystem extends Subsystem {

    protected TalonPIDController controller;
    protected PIDSourceTalon talon;

    public void enablePID() {
        controller.stop();
        controller.enable();
    }

    public boolean isPIDenabled() {
        return controller.isEnabled();
    }

    public void stop() {
        controller.stop();
    }

    public boolean onTarget() {
        return controller.onTarget();
    }

    public void move(double speed) {
        talon.set(-Math.signum(speed) * speed * speed);
    }

    public abstract class TalonPIDController extends PIDController {
        public TalonPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
            super(p, i, d, f, src, out);
        }

        public void stop() {
            setSetpoint(m_pidInput.pidGet());
        }
    }

    public void setP(double p) {
        controller.setP(p);
    }

    public void setI(double i) {
        controller.setI(i);
    }

    public void setD(double d) {
        controller.setD(d);
    }

    public void setF(double f) {
        controller.setF(f);
    }

}
