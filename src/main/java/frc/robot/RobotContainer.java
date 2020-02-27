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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.superstructure.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // define subsystems 
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain s_drive;
  public final Flywheel s_flywheel;
  public static Intake s_intake;

  // create commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); 
  public final Shoot c_shoot;
  
  // create joysticks
  public static final XboxController driveController = new XboxController(OI.DRIVE_CONTROLLER);
  public static final XboxController operatorController = new XboxController(OI.OPERATOR_CONTROLLER);

  public RobotContainer() {
    // define subsystems
 		s_drive = new Drivetrain();
    s_flywheel = new Flywheel();
    s_intake = new Intake();

    // define commands
    c_shoot = new Shoot(s_flywheel);

    // set default commands
    s_flywheel.setDefaultCommand(c_shoot);
    
		s_drive.setDefaultCommand(new RunCommand(() -> s_drive.curveDrive(
				OI.getTriggerOutput(driveController),
				OI.getLeftJoystickAxis(driveController),
				driveController.getXButton()), s_drive)

    // inline bindings                          
    configureButtonBindings();
  }
  

  private void configureButtonBindings() {

    // intake
    new JoystickButton(operatorController, XboxController.Button.kA.value)
        .whenPressed(new RunCommand(s_intake::startVore, s_intake))
        .whenReleased(new RunCommand(s_intake::stopVore, s_intake));

    // intake unjam
    new JoystickButton(operatorController, XboxController.Button.kY.value)
            .whenPressed(new RunCommand(s_intake::unVore, s_intake))
            .whenReleased(new RunCommand(s_intake::stopVore, s_intake));

  }

	public Command getAutonomousCommand() {
		return m_autoCommand;
	}

}
