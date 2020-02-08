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

	// ramsete
	public static final class RamseteConstants {

		// ramsete constants
		public static final double RAMSETE_B = 2;
		public static final double RAMSETE_ZETA = 0.7;

		public static final double kP_VEL = 0.00001;

		public static final double MAX_VOLTAGE_CONSTRAINT = 40;

		// drivetrain constants

		// kS in volts, kV in volts * seconds/meters, kA in volts * seconds^2/meters
		public static final double kS = 0;
		public static final double kV = 0;
		public static final double kA = 0;

		public static final double MAX_METERS_PER_SECOND = 5.45592;
		public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 420.420;
	}

	// camera
	public static final class Camera {

	}

	// controllers

}
