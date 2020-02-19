/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Hopper;

/**
 * An example command that uses an example subsystem.
 */
public class Shoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheel flywheel;
  private final Hopper hopper;

  /**
   * Creates a new ExampleCommand.
   *
   * @param flywheel The subsystem used by this command.
   */
  public Shoot(Flywheel flywheel, Hopper hopper) {
    this.flywheel = flywheel;
    this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.enableController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(flywheel.isAtSpeed()) {
      hopper.startSpit();
    } else {
      hopper.endSpit();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
