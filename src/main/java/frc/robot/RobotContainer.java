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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendAndIntake;
import frc.robot.commands.ExtendAndReverseIntake;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Intake;

public class RobotContainer {

  // define subsystems 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static Intake s_intake;

  // create commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final ExtendAndIntake c_intake;
  private final ExtendAndReverseIntake c_reverseIntake;

  //create joysticks
  public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
  public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  public RobotContainer() {
    s_intake = new Intake();

    c_intake = new ExtendAndIntake(s_intake);
    c_reverseIntake = new ExtendAndReverseIntake(s_intake);

    configureButtonBindings();
  }
  

  private void configureButtonBindings() {

    // extend and intake
    new JoystickButton(operatorController, XboxController.Button.kA.value)
            .whenHeld(c_intake);

    // extend and reverse intake
    new JoystickButton(driveController, XboxController.Button.kY.value)
            .whenHeld(c_reverseIntake);

  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

}
