/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

	// drivetrain
	public static final class Drivetrain {

		// motors
		public static final int FL_ID = 5;
		public static final int FR_ID = 1;
		public static final int BR_ID = 4;
		public static final int BL_ID = 8;

		// configuration
		public static final int	CURRENT_LIMIT = 40;

	}

	// flywheel
	public static final class Flywheel {

		// motors
		public static final int MAIN_ID = 2;
		public static final int SECONDARY_ID = 3;

		// encoders
		public static final int ENCODER_A = 1234;
		public static final int ENCODER_B = 4321;
		public static final boolean reverseEncoder = false;

		// flywheel speed (rotations per minute)
		public static final double SPEED = 60;

		// don't tune these--documentation stated constants for entering rpm
		public static final int TICKS_PER_ROTATION = 4096; // ticks per one encoder rotation
		public static final double SETPOINT_CONSTANT = 0.001667; // 100 ms / 1 min

		// pid constants
		public static final double kP = 0;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0;
		public static final double kV = 0;
		public static final double kA = 0;
		public static final double ERROR_TOLERANCE = 0;

		// timeout value for parameter configs
		public static final int CONFIG_TIMEOUT = 30;

	}

	// camera
	public static final class Camera {

	}

	// climb
	public static final class ClimbConstants {
		public static final int DEPLOY_TALON = 6;
		public static final int WINCH_TALON = 7;
		public static final int WINCH_VICTOR = 10;

		// climb motor max speeds
		public static final int LIFT_SPEED = 1;
		public static final int WINCH_SPEED = -1;

		// climb motor inverted
		public static final boolean TALON_INVERTED = false;
		public static final boolean VICTOR_INVERTED = true;

		// climb encoder CAN ID
		public static final int CLIMB_CAN_ID = 42;

		// climb PID constants (need to tune)
		public static final int ARM_kP = 0;
		public static final int ARM_kI = 0;
		public static final int ARM_kD = 0;

		// climb motion profiling constants (need to tune)
		public static final int MAX_VELOCITY_RAD_PER_SEC = 69;
		public static final int MAX_ACCEL = 69;

		// Encoder distance per pulse
		public static final int PULSES_PER_ROTATION = 4096;
		public static final double DISTANCE_PER_PULSE = (2.0 * Math.PI) / PULSES_PER_ROTATION;

		// starting arm position (need to set)
		public static final double BASE_POSE = 0;

		// feedforward constants; kS = volts; kG = volts (counteracts gravity); kV = volts * sec / radians; kA = volts * sec^2 / radians
		// (need to characterize)
		public static final double kS = 0.0;
		public static final double kG = 0.0;
		public static final double kV = 0.0;
		public static final double kA = 0.0;

		// elevator linear speed in inches (please tell me when we know these things)
		public static final double CLIMB_ELEVATOR_DISTANCE_PER_ROTATION = 0;

		// climb heights
		public static final double MAX_HEIGHT = 78.875;
		public static final double MID_HEIGHT = 66.0;
		public static final double MIN_HEIGHT = 50.25;

	}

	// controllers

}
