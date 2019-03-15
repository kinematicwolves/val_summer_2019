package org.usfirst.frc5577.GearsBot.subsystems;

import org.usfirst.frc5577.GearsBot.RobotMap;
import org.usfirst.frc5577.GearsBot.Utility;
import org.usfirst.frc5577.GearsBot.commands.ManualElevatorControl;
import org.usfirst.frc5577.GearsBot.subsystems.pid.TalonPIDSubsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Elevator extends TalonPIDSubsystem {

	public static final double Kp_default = 0.0001;
	public static final double Ki_default = 0.0;
	public static final double Kd_default = 0.0;
	// Kf may need to be adjusted if the elevator does not hold steady state
	public static final double Kf_default = 0.1;

	private double Kp;
	private double Ki;
	private double Kd;
	private double Kf;

	// Height units in inches
	public static final double MINIMUM_HEIGHT = 0.0;
	public static final double MAXIMUM_HEIGHT = 37.0; // Start with 40 inches, increase from there
	public static final double HEIGHT_TOLERANCE = 2.0;
	public static final double STARTING_HEIGHT = 0.0;

	/**
	 * The sprocketScaleFactor accounts for how much the output of the gearbox must
	 * spin to power the rotation of the elevator spool itself. If the gear at the
	 * output of the gearbox has 15 teeth, and the gear connected via chain has 32
	 * teeth, then the scale factor is 15/32.
	 */
	private static final double sprocketScaleFactor = (15.0 / 32.0);
	private static final double spoolDiameter = 1.25; // Inches!
	private static final double spoolCircumference = Math.PI * spoolDiameter; // Inches in 1 revolution
	private static final double countsPerRevolution = 4096;
	private static final double inchesPerEncoderCount = (spoolCircumference / countsPerRevolution)
			* sprocketScaleFactor;

	private ShuffleboardTab pid_tab_elevator;
	private NetworkTableEntry nt_Kp_elevator;
	private NetworkTableEntry nt_Ki_elevator;
	private NetworkTableEntry nt_Kd_elevator;
	private NetworkTableEntry nt_Kf_elevator;

	public static double toInches(double counts) {
		return counts * inchesPerEncoderCount;
	}

	public static double toCounts(double inches) {
		return inches / inchesPerEncoderCount;
	}

	public Elevator() {
		talon = RobotMap.elevatorTalonSRX;
		talon.setName("ElevatorTalon");
		talon.setNeutralMode(NeutralMode.Brake);
		talon.setInverted(false);

		controller = new ElevatorPIDController(Kp_default, Ki_default, Kd_default, Kf_default, talon, talon);
		controller.setAbsoluteTolerance(HEIGHT_TOLERANCE);

		pid_tab_elevator = Shuffleboard.getTab("Elevator PID");
		nt_Kp_elevator = pid_tab_elevator.add("kP", Kp_default).getEntry();
		nt_Ki_elevator = pid_tab_elevator.add("kI", Ki_default).getEntry();
		nt_Kd_elevator = pid_tab_elevator.add("kD", Kd_default).getEntry();
		nt_Kf_elevator = pid_tab_elevator.add("kF", Kf_default).getEntry();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator height", getHeight());
		SmartDashboard.putNumber("Elevator height setpoint", getSetpointHeight());
		SmartDashboard.putNumber("Elevator height (actual) counts",
				talon.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Elevator height setpoint counts", getSetpointCounts());
		SmartDashboard.putBoolean("Elevator PID On", controller.isEnabled());
		SmartDashboard.putNumber("Elevator error", controller.getError());
		SmartDashboard.putNumber("Elevator motor out", talon.get());

		Kp = nt_Kp_elevator.getDouble(0);
		Ki = nt_Ki_elevator.getDouble(0);
		Kd = nt_Kd_elevator.getDouble(0);
		Kf = nt_Kf_elevator.getDouble(0);

		setP(Kp);
		setI(Ki);
		setD(Kd);
		setF(Kf);
	}

	public void moveSetpoint(double speed) {
		double height = getSetpointHeight();
		height += speed;
		setHeight(height);
	}

	public void setHeight(double inches) {
		inches = Utility.clamp(inches, MINIMUM_HEIGHT, MAXIMUM_HEIGHT);
		double counts = toCounts(inches - STARTING_HEIGHT); // Measures the difference from the starting height
		controller.setSetpoint(counts);
	}

	public double getHeight() {
		int quadraturePosition = talon.getSensorCollection().getQuadraturePosition();
		return STARTING_HEIGHT + toInches(quadraturePosition);
	}

	public double getSetpointHeight() {
		double counts = controller.getSetpoint();
		return STARTING_HEIGHT + toInches(counts);
	}

	public double getSetpointCounts() {
		return controller.getSetpoint();
	}

	public void moveLift(double speed) {
		RobotMap.elevatorTalonSRX.set(-speed);
	}

	public void stop() {
		RobotMap.elevatorTalonSRX.set(ControlMode.PercentOutput, 0);
	}

	public class ElevatorPIDController extends TalonPIDController {
		ElevatorPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
			super(p, i, d, f, src, out);
		}

		@Override
		protected double calculateFeedForward() {
			return getF(); // Should probably be a constant force from gravity to overcome
		}
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new ManualElevatorControl());
	}

}
