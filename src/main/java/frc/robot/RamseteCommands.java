package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class RamseteCommands {
	// sets voltage constraint so you dont over accelerate
	public static DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
			new SimpleMotorFeedforward(Constants.RamseteConstants.kS,
					Constants.RamseteConstants.kV, Constants.RamseteConstants.kA), RobotContainer.s_drive.driveKinematics,
			Constants.RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

	// this class is the entire path/sequence
	public static class ExamplePath {
		// returns the SequentialCommandGroup used in auto
		public static SequentialCommandGroup fullPath() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					Paths.Example()[1],
					RobotContainer.s_drive::getPose,
					new RamseteController(Constants.RamseteConstants.RAMSETE_B, Constants.RamseteConstants.RAMSETE_ZETA),
					new SimpleMotorFeedforward(Constants.RamseteConstants.kS, Constants.RamseteConstants.kV, Constants.RamseteConstants.kA),
					RobotContainer.s_drive.driveKinematics,
					RobotContainer.s_drive::getWheelSpeeds,
					new PIDController(Constants.RamseteConstants.kP_VEL, 0, 0),
					new PIDController(Constants.RamseteConstants.kP_VEL, 0, 0),
					// return the volts
					RobotContainer.s_drive::tankDriveVolts,
					RobotContainer.s_drive
			);
			return new SequentialCommandGroup(ramseteCommand);
		}
	}
}
