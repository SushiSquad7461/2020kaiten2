/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
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
				.whenPressed(new ParallelCommandGroup(
						new RunCommand(s_visionAlign::alignRobot),
						new ConditionalCommand(
								new ConditionalCommand(
										new ConditionalCommand(
												new RunCommand(s_hopper::startSpit),
												new SequentialCommandGroup(
														new RunCommand(s_hopper::reverseSpit),
														new WaitCommand(0.5)
												),
												s_hopper::isCurrentSpiked
										),
										new InstantCommand(s_hopper::endSpit),
										s_visionAlign::checkAdjacent
								)
						)
				))
				.whenReleased(new ParallelCommandGroup(
						new InstantCommand(s_visionAlign::cancelAlign),
						new InstantCommand(s_hopper::endSpit)
				));

	}

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
