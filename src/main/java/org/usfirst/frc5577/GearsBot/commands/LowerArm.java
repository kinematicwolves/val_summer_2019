package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;
import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class LowerArm extends Command {

    private double speed = 0.7;
    private double time = 0;

    public LowerArm() {
        this(0.8);
    }

    public LowerArm(double speed) {
        requires(Robot.arm);
        this.speed = speed;
    }

    public LowerArm(double speed, double time) {
        this(speed);
        this.time = time;
    }

    @Override
    protected void initialize() {
        Robot.arm.lowerArm(speed);
    }

    @Override
    protected void execute() {
        System.out.println("The speed of the arm motion is " + RobotMap.redTalonSRX.get());
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void interrupted() {
        // Robot.arm.setSensorPosition();
        Robot.arm.lowerArm(0);
    }

}
