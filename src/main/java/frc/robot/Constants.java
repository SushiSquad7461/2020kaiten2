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
		public static final int FL_ID = 21;
		public static final int FR_ID = 22;
		public static final int BR_ID = 23;
		public static final int BL_ID = 24;

		// encoders
		public static final int ENCODER_LEFT = 3;
		public static final int ENCODER_RIGHT = 4;

		// open loop configuration
		public static final int	CURRENT_LIMIT = 35;
		public static final int	OPEN_LOOP_RAMP = 0;
		public static final double SLOW_SPEED = 0.1;

		public static final double CONTROLLER_LINEAR_SCALING = 75;
		public static final double CONTROLLER_ANGULAR_SCALING = 100;

		// pid constants
		public static final double LEFT_kP = 0.356;
		public static final double LEFT_kI = 0;
		public static final double LEFT_kD = 0;

		public static final double LEFT_kS = 0.166;
		public static final double LEFT_kV = 0.0315;
		public static final double LEFT_kA = 0.00743;

		public static final double RIGHT_kP = 0.356;
		public static final double RIGHT_kI = 0;
		public static final double RIGHT_kD = 0;

		public static final double RIGHT_kS = 0.166;
		public static final double RIGHT_kV = 0.0315;
		public static final double RIGHT_kA = 0.00743;
		
		// closed loop constants
		public static final double wheelRadius = 3.0;
		public static final double trackWidth = 0.6420;

		public static final double encoderResolution = 0;

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

}
