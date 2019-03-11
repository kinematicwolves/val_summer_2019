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
    public static final double Kp_default = 0.0001;
    public static final double Ki_default = 0.0;
    public static final double Kd_default = 0.0;
    public static final double Kf_default = 0.1;

    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    // Zero is with the arm straight horizontal
    public static final double MINIMUM_ANGLE = 0.0;
    public static final double MAXIMUM_ANGLE = 120.0;
    public static final double ANGLE_TOLERANCE = 2.0;
    public static final double STARTINGANGLE = 0.0;

    // Adjust gearBoxScaleFactor based on the gear box ratio. 9:1 gear box output
    // would probably be (1.0 / 9.0).
    // This number will likely be less than 1.
    private static final double gearBoxScaleFactor = (40.0 / 60);
    private static final double degreesPerEncoderCount = (360.0 / 4096) * gearBoxScaleFactor;

    private ShuffleboardTab pid_tab;
    private NetworkTableEntry nt_Kp;
    private NetworkTableEntry nt_Ki;
    private NetworkTableEntry nt_Kd;
    private NetworkTableEntry nt_Kf;

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
        talon.setInverted(false);

        controller = new VerticalArmPIDController(Kp_default, Ki_default, Kd_default, Kf_default, talon, talon);
        controller.setAbsoluteTolerance(ANGLE_TOLERANCE); // TODO set tolerance in counts

        pid_tab = Shuffleboard.getTab("Arm PID");
        nt_Kp = pid_tab.add("kP", Kp_default).getEntry();
        nt_Ki = pid_tab.add("kI", Ki_default).getEntry();
        nt_Kd = pid_tab.add("kD", Kd_default).getEntry();
        nt_Kf = pid_tab.add("kF", Kf_default).getEntry();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putNumber("Arm Angle counts", talon.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putBoolean("Arm PID On", controller.isEnabled());
        SmartDashboard.putNumber("Arm error", controller.getError());
        SmartDashboard.putNumber("Arm Setpoint", getSetpointDegrees());
        SmartDashboard.putNumber("Arm motor out", talon.get());

        Kp = nt_Kp.getDouble(0);
        Ki = nt_Ki.getDouble(0);
        Kd = nt_Kd.getDouble(0);
        Kf = nt_Kf.getDouble(0);

        setP(Kp);
        setI(Ki);
        setD(Kd);
        setF(Kf);
    }

    public void setAngle(double angle) {
        angle = Utility.clamp(angle, MINIMUM_ANGLE, MAXIMUM_ANGLE);
        double counts = toCounts(STARTINGANGLE - angle);
        controller.setSetpoint(counts);
    }

    public double getAngle() {
        int quadraturePosition = talon.getSensorCollection().getQuadraturePosition();
        return STARTINGANGLE - toDegrees(quadraturePosition);
    }

    public void moveSetpoint(double speed) {
        double ang = getSetpointDegrees();
        ang += speed;
        setAngle(ang);
    }

    public double getSetpointDegrees() {
        double counts = controller.getSetpoint();
        return STARTINGANGLE - toDegrees(counts);
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
