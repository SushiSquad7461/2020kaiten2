package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

	// controllers
	public static final int DRIVE_CONTROLLER = 1;
	public static final int OPERATOR_CONTROLLER = 2;

	// cubed -1 to 1 output from trigger controllers
	public static double getTriggerOutput(XboxController controller) {
		return Math.pow(controller.getTriggerAxis(GenericHID.Hand.kRight) - controller.getTriggerAxis(GenericHID.Hand.kLeft), 3);
	}

	// joystick left-hand x axis
	public static double getLeftJoystickAxis(XboxController controller) {
		return controller.getX(GenericHID.Hand.kLeft);
	}

	// joystick right-hand x axis
	public static double getRightJoystickAxis(XboxController controller) {
		return controller.getX(GenericHID.Hand.kRight);
	}

}