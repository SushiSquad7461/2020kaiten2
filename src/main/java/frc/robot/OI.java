package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {

	// controllers
	public static final int DRIVE_CONTROLLER = 0;
	public static final int OPERATOR_CONTROLLER = 1;

	// configuration constants
	public static final double TRIGGER_TOLERANCE = 0.3;

	// boolean from trigger input
	public static boolean getTriggerBoolean(XboxController controller, GenericHID.Hand hand) {
    return controller.getTriggerAxis(hand) > TRIGGER_TOLERANCE;
	}

	// cubed -1 to 1 output from trigger controllers
	public static double getTriggerOutput(XboxController controller) {
		return Math.pow(controller.getTriggerAxis(GenericHID.Hand.kRight) - controller.getTriggerAxis(GenericHID.Hand.kLeft), 3);
	}

	// joystick left-hand x axis
	public static double getLeftJoystickAxis(XboxController controller) {
		return Math.pow(controller.getX(GenericHID.Hand.kLeft), 3);
	}

	// joystick right-hand x axis
	public static double getRightJoystickAxis(XboxController controller) {
		return controller.getX(GenericHID.Hand.kRight);
	}

}