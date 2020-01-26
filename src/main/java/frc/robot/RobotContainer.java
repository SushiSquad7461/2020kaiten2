/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // define subsystems and commands

  public static Climb s_climb;
  public static ExampleSubsystem m_exampleSubsystem;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // instantiate controllers
  public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  public RobotContainer() {

    configureButtonBindings();
    s_climb = new Climb();
  }

  private void configureButtonBindings() {
    new JoystickButton(operatorController, XboxController.Button.kA.value).whenPressed(s_climb::startDeployClimbArm)
            .whenReleased(s_climb::stopDeployClimbArm);
    new JoystickButton(operatorController, XboxController.Button.kB.value).whenPressed(s_climb::startWinch)
            .whenReleased(s_climb::stopWinch);
    new JoystickButton(operatorController, operatorController.getPOV(0)).whenPressed(() -> s_climb.calculateInput(5));
    new JoystickButton(operatorController, operatorController.getPOV(90)).whenPressed(() -> s_climb.calculateInput(4));
    new JoystickButton(operatorController, operatorController.getPOV(270)).whenPressed(() -> s_climb.calculateInput(3));
    new JoystickButton(operatorController, operatorController.getPOV(180)).whenPressed(s_climb::dropElevator);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}
