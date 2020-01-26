/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Ramsete;
import frc.robot.subsystems.ExampleSubsystem;

public class RobotContainer {
	// define subsystems and commands
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	public final Drivetrain s_drive = new Drivetrain();

	private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
	private final Ramsete ramsete = new Ramsete(s_drive);

	public RobotContainer() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
	}

	public Command getAutonomousCommand() {
		return new SequentialCommandGroup(
				ramsete.ramseteCommand
				);
	}
}
