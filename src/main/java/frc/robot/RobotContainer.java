/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.Constants;

public class RobotContainer {

  // define subsystems 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Intake s_intake;

  // create commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); 

  //create joysticks
  public static final XboxController driveController = new XboxController(5);
  public static final XboxController operatorController = new XboxController(5);
  

  public RobotContainer() {
    s_intake = new Intake();
    configureButtonBindings();
  }
  

  private void configureButtonBindings() {

    new JoystickButton(driveController, XboxController.Button.kA.value)
        .whenPressed(new RunCommand(s_intake::startVore))
        .whenReleased(new RunCommand(s_intake::stopVore));

  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}
