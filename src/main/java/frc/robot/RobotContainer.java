/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // define subsystems and commands
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Flywheel s_flywheel;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final Shoot c_shoot;

  // create joysticks
  public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
  public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  public RobotContainer() {
    // define subsystems
    s_flywheel = new Flywheel();

    // define commands
    c_shoot = new Shoot(s_flywheel);

    // set default commands
    s_flywheel.setDefaultCommand(c_shoot);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}
