/*
    This command will pick up all the balls it sees until it's done running.
    It starts with the ball closest to the middle of the vision.
    That might be a bad way to do it if we want to get all the balls in sight, 
    someone smarter than me might want to think about that once this is all
    up and working.
*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis.*;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;

// haha penis joke
public class EatMyBalls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain dt;
  private final Intake intake;
  private NetworkTableInstance inst;
  private NetworkTable data, camera;
  private NetworkTableEntry pitch;

  public EatMyBalls(Drivetrain dt, Intake intake) {
    this.dt = dt;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inst = NetworkTableInstance.getDefault();
    data = inst.getTable("chameleon-vision");
    camera = data.getSubTable(Constants.Camera.NAME);
    inst.startClientTeam(7461);
    inst.startDSClient();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = camera.getEntry("targetPitch").getDouble(0);
    intake.startEat();
    dt.closedCurveDrive(Constants.BallFinder.DRIVE_SPEED, -pitch, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      dt.stop();
      intake.stopEat();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return camera.getEntry("isValid").getBoolean(false);
  }
}
