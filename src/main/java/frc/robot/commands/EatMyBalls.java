/*
    This command will pick up all the balls it sees until it's done running.
    It starts with the ball closest to the middle of the vision.
    That might be a bad way to do it if we want to get all the balls in sight, 
    someone smarter than me might want to think about that once this is all
    up and working.
*/

package frc.robot.commands;

import frc.robot.subsystems.Chassis.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

// haha penis joke
public class EatMyBalls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain dt;

  public EatMyBalls(Drivetrain dt) {
    this.dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
