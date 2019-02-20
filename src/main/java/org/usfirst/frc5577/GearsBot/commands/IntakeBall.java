package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;
import org.usfirst.frc5577.GearsBot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeBall extends Command {

    private double speed = 0.8;

    public IntakeBall() {
        this(0.8);
    }

    public IntakeBall(double speed) {
        requires(Robot.intake);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
        Robot.intake.intakeBall(speed);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted() {
        Robot.intake.intakeBall(0);
    }
}