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
		public static final int MAIN_ID = 11;
		public static final int SECONDARY_ID = 10;

		public static final boolean MAIN_INVERTED = false;
		public static final boolean SECONDARY_INVERTED = true;

		// encoders
		public static final int ENCODER = 60;

		// encoders
		public static final int ENCODER_A = 60;
		public static final int ENCODER_B = 4321;
		public static final boolean ENCODER_REVERSE_DIRECTION = false;

		// flywheel speed (rotations per minute)
		public static final double SPEED = 6000;

		// don't tune these--documentation stated constants for entering rpm
		public static final int TICKS_PER_ROTATION = 4096; // ticks per one encoder rotation
		public static final double SETPOINT_CONSTANT = 0.001667; // 100 ms / 1 min

		// pid constants
		public static final double kP = 0.00002;
		public static final double kI = 0;
		public static final double kD = 0;

		public static final double kS = 0.452;
		public static final double kV = 0.000334;
		public static final double kA = 0;
		public static final double ERROR_TOLERANCE = 0;

		// timeout value for parameter configs
		public static final int CONFIG_TIMEOUT = 30;

	}

	// hopper
	public static final class Hopper {
		public static final int FAST_ID = 1;
		public static final int SLOW_ID = 6;

		public static final double MAX_SPEED = 0.6;
		public static final double SLOW_SPEED = 0.2;
		public static final double REVERSE_SPEED = -0.6;

		public static final int CONFIG_TIMEOUT = 30;
		public static final int CURRENT_SPIKE = 70;
	}

	// camera
	public static final class Camera {

	}

}
