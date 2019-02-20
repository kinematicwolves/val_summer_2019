package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class HoldArm extends Command {

    public HoldArm() {
        requires(Robot.arm);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        // Robot.arm.hold();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
