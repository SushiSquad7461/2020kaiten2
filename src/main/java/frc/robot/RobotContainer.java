/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Chassis.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.Constants;

public class RobotContainer {

  // define subsystems 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Intake s_intake;
  private final Drivetrain s_drive = new Drivetrain();

  // create commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); 

	XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
	XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  public RobotContainer() {
    s_intake = new Intake();
    configureButtonBindings();

		s_drive.setDefaultCommand(new RunCommand(() -> s_drive.closedCurveDrive(
				OI.getTriggerOutput(driveController),
				OI.getLeftJoystickAxis(driveController),
				driveController.getXButton()), s_drive)
		);

  }
  

  private void configureButtonBindings() {
    new JoystickButton(driveController, XboxController.Button.kA.value)
        .whenPressed(s_intake::startEat)
        .whenReleased(s_intake::stopEat); 
  }

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
