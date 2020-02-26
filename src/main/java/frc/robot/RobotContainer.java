/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

	// define subsystems and commands
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final Drivetrain s_drive;

	private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

	XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
	XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

	public RobotContainer() {

		s_drive = new Drivetrain();

		s_drive.setDefaultCommand(new RunCommand(() -> s_drive.closedCurveDrive(
				OI.getTriggerOutput(driveController),
				OI.getLeftJoystickAxis(driveController),
				driveController.getXButton()), s_drive)
		);

		configureButtonBindings();

	}

	private void configureButtonBindings() {
	}

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
