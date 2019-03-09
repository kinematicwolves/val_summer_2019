// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc5577.GearsBot;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc5577.GearsBot.buttons.DPadButton;
import org.usfirst.frc5577.GearsBot.buttons.DPadButton.Direction;
import org.usfirst.frc5577.GearsBot.buttons.JoystickAnalogButton;
import org.usfirst.frc5577.GearsBot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public final double ANGLE_MOTOR_SPEED = 0.5;
	public final double SHOOTER_GATE_POSITION = 0.5;

	public final int A_BUTTON = 1;
	public final int B_BUTTON = 2;
	public final int X_BUTTON = 3;
	public final int Y_BUTTON = 4;
	public final int L_BUMPER = 5;
	public final int R_BUMPER = 6;
	public final int BACK_BUTTON = 7;
	public final int START_BUTTON = 8;
	public final int LEFT_STICK_BUTTON = 9;
	public final int RIGHT_STICK_BUTTON = 10;

	public final int LEFT_AXIS_X = 0;
	public final int LEFT_AXIS_Y = 1;
	public final int LEFT_TRIGGER_AXIS = 2;
	public final int RIGHT_TRIGGER_AXIS = 3;
	public final int RIGHT_AXIS_X = 4;
	public final int RIGHT_AXIS_Y = 5;
	public final int DPAD_AXIS = 6;

	public Joystick driverController; // This is for xbox controller one or player one
	public Joystick manipulatorController; // This is for xbox controller two or player two

	private List<Button> buttons = new ArrayList<>();

	public static OI instance;

	public static OI GetInstance() {
		if (instance == null) {
			instance = new OI();
		}
		return instance;
	}

	public Joystick getJoystick() {
		return driverController;
	}

	public OI() {

		/*
		 * You can uncomment (Ctrl + /) lines of code below to enable buttons on either
		 * the driver controller or the manipulator controller.
		 * 
		 * These are pretty easy to configure, so if you have an idea for improved
		 * controls, let's try it! :)
		 */

		// Driver Controller Setup
		driverController = new Joystick(0);

		Button rBumperDriver = new JoystickButton(driverController, R_BUMPER);
		buttons.add(rBumperDriver);
		// Button dPadUp = new DPadButton(driverController, Direction.UP);
		// Button dPadLeft = new DPadButton(driverController, Direction.LEFT);
		// Button dPadRight = new DPadButton(driverController, Direction.RIGHT);

		rBumperDriver.whenPressed(new ShiftGear());

		// Manipulator Controller Setup
		manipulatorController = new Joystick(1);

		Button aButton = new JoystickButton(manipulatorController, A_BUTTON);
		buttons.add(aButton);
		Button bButton = new JoystickButton(manipulatorController, B_BUTTON);
		buttons.add(bButton);
		Button xButton = new JoystickButton(manipulatorController, X_BUTTON);
		buttons.add(xButton);
		Button yButton = new JoystickButton(manipulatorController, Y_BUTTON);
		buttons.add(yButton);
		Button lBumperManipulator = new JoystickButton(manipulatorController, L_BUMPER);
		buttons.add(lBumperManipulator);
		Button lTriggerManipulator = new JoystickAnalogButton(manipulatorController, LEFT_TRIGGER_AXIS);
		buttons.add(lTriggerManipulator);
		Button rBumperManipulator = new JoystickButton(manipulatorController, R_BUMPER);
		buttons.add(rBumperManipulator);
		Button rTriggerManipulator = new JoystickAnalogButton(manipulatorController, RIGHT_TRIGGER_AXIS);
		buttons.add(rTriggerManipulator);
		Button r3Manipulator = new JoystickButton(manipulatorController, RIGHT_STICK_BUTTON);
		buttons.add(r3Manipulator);
		Button dPadUp = new DPadButton(manipulatorController, Direction.UP);
		buttons.add(dPadUp);
		Button dPadDown = new DPadButton(manipulatorController, Direction.DOWN);
		buttons.add(dPadDown);

		dPadUp.whileHeld(new ElevatorUp(0.85));
		dPadDown.whileHeld(new ElevatorDown(0.60));

		lBumperManipulator.whenPressed(new ShiftWrist());
		lTriggerManipulator.whenPressed(new ShiftHatchPanel());

		xButton.whileHeld(new IntakeBall(0.8));

		bButton.whileHeld(new ShootBall(0.8));
	}

	public void close() {
		for (Button b : buttons) {
			b.close();
		}
	}

}