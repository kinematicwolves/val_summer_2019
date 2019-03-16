package org.usfirst.frc5577.GearsBot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc5577.GearsBot.RobotMap;
import org.usfirst.frc5577.GearsBot.Utility;
import org.usfirst.frc5577.GearsBot.commands.ManualArmControl;
import org.usfirst.frc5577.GearsBot.subsystems.pid.TalonPIDSubsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends TalonPIDSubsystem {

    // Adjust Kp between 0.0001 and 0.0005, based on the arm. Greater than 0.0005
    // seems to do wacky things.
    public static final double Kp_default = 0.00022;
    public static final double Ki_default = 0.0;
    public static final double Kd_default = 0.0;
    // Kf may need to be adjusted if the arm does not hold steady state at 0°
    public static final double Kf_default = 0.2;

    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    // Zero is with the arm straight horizontal
    public static final double MINIMUM_ANGLE = -30.0;
    public static final double MAXIMUM_ANGLE = 90.0;
    public static final double ANGLE_TOLERANCE = 2.0;
    public static final double STARTING_ANGLE = 90.0;

    /**
     * The sprocketScaleFactor accounts for how much the output of the gearbox must
     * spin to power the rotation of arm itself. If the gear at the output of the
     * gearbox has 15 teeth, and the gear connected via chain has 23 teeth, then the
     * scale factor is 15/23.
     */
    private static final double sprocketScaleFactor = (15.0 / 23.0);
    private static final double degreesPerEncoderCount = (360.0 / 4096) * sprocketScaleFactor;

    private ShuffleboardTab pid_tab_arm;
    private NetworkTableEntry nt_Kp_arm;
    private NetworkTableEntry nt_Ki_arm;
    private NetworkTableEntry nt_Kd_arm;
    private NetworkTableEntry nt_Kf_arm;

    public static double toDegrees(double counts) {
        return counts * degreesPerEncoderCount;
    }

    public static double toCounts(double degrees) {
        return degrees / degreesPerEncoderCount;
    }

    public Arm() {
        talon = RobotMap.armTalonSRX;
        talon.setName("ArmTalon");
        talon.setNeutralMode(NeutralMode.Brake);
        talon.setInverted(true);

        controller = new VerticalArmPIDController(Kp_default, Ki_default, Kd_default, Kf_default, talon, talon);
        controller.setAbsoluteTolerance(ANGLE_TOLERANCE); // TODO set tolerance in counts

        pid_tab_arm = Shuffleboard.getTab("Arm PID");
        nt_Kp_arm = pid_tab_arm.add("kP", Kp_default).getEntry();
        nt_Ki_arm = pid_tab_arm.add("kI", Ki_default).getEntry();
        nt_Kd_arm = pid_tab_arm.add("kD", Kd_default).getEntry();
        nt_Kf_arm = pid_tab_arm.add("kF", Kf_default).getEntry();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm angle", getAngle());
        SmartDashboard.putNumber("Arm setpoint degrees", getSetpointDegrees());
        SmartDashboard.putNumber("Arm angle (actual) counts", talon.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber("Arm angle setpoint counts", getSetpointCounts());
        SmartDashboard.putNumber("Feed forward value", getFeedForward());
        SmartDashboard.putBoolean("Arm PID on", controller.isEnabled());
        SmartDashboard.putNumber("Arm error", controller.getError());
        SmartDashboard.putNumber("Arm motor out", talon.get());

        Kp = nt_Kp_arm.getDouble(0);
        Ki = nt_Ki_arm.getDouble(0);
        Kd = nt_Kd_arm.getDouble(0);
        Kf = nt_Kf_arm.getDouble(0);

        setP(Kp);
        setI(Ki);
        setD(Kd);
        setF(Kf);
    }

    public void moveSetpoint(double speed) {
        double ang = getSetpointDegrees();
        ang += speed;
        setAngle(ang);
    }

    public void setAngle(double angle) {
        angle = Utility.clamp(angle, MINIMUM_ANGLE, MAXIMUM_ANGLE);
        double counts = toCounts(angle - STARTING_ANGLE); // Measures the difference from the starting angle
        controller.setSetpoint(counts);
    }

    public double getAngle() {
        int quadraturePosition = talon.getSensorCollection().getQuadraturePosition();
        return STARTING_ANGLE + toDegrees(quadraturePosition);
    }

    public double getSetpointDegrees() {
        double counts = controller.getSetpoint();
        return STARTING_ANGLE + toDegrees(counts);
    }

    public double getSetpointCounts() {
        return controller.getSetpoint();
    }

    public double getFeedForward() {
        return 0.1 * Math.cos(Math.toRadians(getAngle()));
    }

    public class VerticalArmPIDController extends TalonPIDController {
        VerticalArmPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
            super(p, i, d, f, src, out);
        }

        @Override
        protected double calculateFeedForward() {
            return getF() * Math.cos(Math.toRadians(getAngle()));
        }

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ManualArmControl());
    }

}
