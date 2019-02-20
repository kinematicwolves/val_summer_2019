package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;
import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class LiftArm extends Command {

    private double speed = 0.8;

    public LiftArm() {
        this(0.8);
    }

    public LiftArm(double speed) {
        requires(Robot.arm);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        Robot.arm.liftArm(speed);
    }

    @Override
    protected void execute() {
        System.out.println("The speed of the arm motion is " + RobotMap.redTalonSRX.get());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {
        // Robot.arm.setSensorPosition();
        Robot.arm.stop();
    }

}
