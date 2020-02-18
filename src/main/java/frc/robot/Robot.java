/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Flywheel;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    autoChooser.setDefaultOption("Example path", RamseteCommands.ExamplePath.fullAutoSequence());
    autoChooser.addOption("o, s8, mTr, 8b", RamseteCommands.Offensive1.fullAutoSequence());
    autoChooser.addOption("o, s8, mM, 8b", RamseteCommands.Offensive2.fullAutoSequence());
    autoChooser.addOption("d, s0, oTr/M, 13b", RamseteCommands.Defensive1.fullAutoSequence());
    autoChooser.addOption("d, s5, oTr, 8b", RamseteCommands.Defensive2.fullAutoSequence());
    autoChooser.addOption("d, s5, mTr, 8b", RamseteCommands.Defensive3.fullAutoSequence());
    autoChooser.addOption("pd, s5, oM, 8b", RamseteCommands.CounterDefensive1.fullAutoSequence());
    autoChooser.addOption("pd, s5, oTr, 8b", RamseteCommands.CounterDefensive2.fullAutoSequence());
    autoChooser.addOption("po, s8, oTr, 8b", RamseteCommands.PseudoOffensive1.fullAutoSequence());
    autoChooser.addOption("po, s8, oM, 8b", RamseteCommands.PseudoOffensive2.fullAutoSequence());
    SmartDashboard.putData("Auto path", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }


  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
