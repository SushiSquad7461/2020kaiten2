/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Hopper;

public class RobotContainer {

	// define subsystems and commands
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final Hopper s_hopper;
	private final Flywheel s_flywheel;
	private final Intake s_intake;

	private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

	// create joysticks
	public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
	//public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

	public RobotContainer() {
		s_hopper = new Hopper();
		s_flywheel = new Flywheel();
		s_intake = new Intake();

		configureButtonBindings();
	}

	private void configureButtonBindings() {

		new JoystickButton(driveController, XboxController.Button.kB.value)
				.whenPressed(new RunCommand(s_intake::startVore, s_intake))
				.whenReleased(new RunCommand(s_intake::stopVore, s_intake));

    // flywheel
    new JoystickButton(driveController, XboxController.Button.kX.value)
			.whenPressed(new InstantCommand(() -> s_flywheel.enableController()))
			.whenReleased(new InstantCommand(() -> s_flywheel.stop()));

	new JoystickButton(driveController, XboxController.Button.kA.value)
			.whenPressed(new ConditionalCommand(
						new RunCommand(s_hopper::startSpit, s_hopper),
						new SequentialCommandGroup(
								new RunCommand(s_hopper::reverseSpit, s_hopper),
								new WaitCommand(0.5)
						),
						s_hopper::isCurrentSpiked)
				)
				.whenReleased(new RunCommand(s_hopper::endSpit, s_hopper));

	}

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
