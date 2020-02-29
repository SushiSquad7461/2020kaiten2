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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // define subsystems and commands

  public static Climb s_climb;
  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // instantiate controllers
  public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
  public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {
    s_climb = new Climb();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driveController, XboxController.Button.kY.value)
            .whenPressed(new InstantCommand(s_climb::climbUp))
            .whenReleased(new InstantCommand(s_climb::stopClimb));

    new JoystickButton(driveController, XboxController.Button.kB.value)
            .whenPressed(new InstantCommand(s_climb::climbDown))
            .whenReleased(new InstantCommand(s_climb::stopClimb));

  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}
