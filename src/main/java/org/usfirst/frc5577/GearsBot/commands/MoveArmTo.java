package org.usfirst.frc5577.GearsBot.commands;

import org.usfirst.frc5577.GearsBot.Robot;
import org.usfirst.frc5577.GearsBot.Utility;

import edu.wpi.first.wpilibj.command.Command;

public class MoveArmTo extends Command {

    private static final double DELTA_ANGLE_MAX = (Robot.arm.MAXIMUM_ANGLE - Robot.arm.MINIMUM_ANGLE) / (4.0 * 50);

    private double speed = 0.8;
    double targetAngle;

    private MoveArmTo() {
        requires(Robot.arm);
    }

    public MoveArmTo(double targetAngle) {
        this();
        this.targetAngle = targetAngle;
    }

    @Override
    protected void initialize() {
        targetAngle = Utility.clamp(targetAngle, Robot.arm.MINIMUM_ANGLE, Robot.arm.MAXIMUM_ANGLE);
    }

    @Override
    protected void execute() {
        // System.out.println("The speed of the arm motion is " +
        // RobotMap.redTalonSRX.get());
        double angleDelta = targetAngle - Robot.arm.getAngle();
        angleDelta = Utility.clamp(angleDelta, -DELTA_ANGLE_MAX, DELTA_ANGLE_MAX);
        Robot.arm.setAngle(Robot.arm.getAngle() + angleDelta);
    }

    @Override
    protected boolean isFinished() {
        return Robot.arm.onTarget();
    }

    @Override
    protected void interrupted() {
        Robot.arm.stop();
    }

}
