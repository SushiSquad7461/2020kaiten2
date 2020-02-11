/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionAlign;

public class RobotContainer {

	// define subsystems and commands
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	public static VisionAlign s_visionAlign;

	private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

	// create joysticks
	public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
	public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

	public RobotContainer() {
		s_visionAlign = new VisionAlign();

		configureButtonBindings();
	}

	private void configureButtonBindings() {

		new JoystickButton(driveController, XboxController.Button.kBumperLeft.value)
				.whenPressed(new RunCommand(s_visionAlign::alignRobot))
				.whenReleased(new InstantCommand(s_visionAlign::cancelAlign));

	}

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
