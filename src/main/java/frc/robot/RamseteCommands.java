package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class RamseteCommands {
	private static Paths paths;
	private DifferentialDriveVoltageConstraint voltageConstraint;
	private static RamseteCommand exampleCommand;

	public RamseteCommands() {
		paths = new Paths();
		// sets voltage constraint so you dont over accelerate
		voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.RamseteConstants.kS,
						Constants.RamseteConstants.kV, Constants.RamseteConstants.kA), RobotContainer.s_drive.driveKinematics,
				Constants.RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

		exampleCommand = new RamseteCommand(
				paths.Example()[0],
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
	}

	public DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return voltageConstraint;
	}

	// this class is the entire path/sequence
	public static class ExamplePath {
		// returns the SequentialCommandGroup used in auto
		public static SequentialCommandGroup fullAutoSequence() {

			return new SequentialCommandGroup(exampleCommand);
		}
	}

	public static class Offensive1 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					paths.Offensive1()[0],
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

	public static class Offensive2 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					paths.Offensive2()[0],
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

	public static class Defensive1 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand followToTrench = new RamseteCommand(
					paths.Defensive1()[0],
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

			RamseteCommand followThroughTrench = new RamseteCommand(
					paths.Defensive1()[1],
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

			RamseteCommand followTrenchToMid = new RamseteCommand(
					paths.Defensive1()[2],
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

			RamseteCommand followThroughMid = new RamseteCommand(
					paths.Defensive1()[3],
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

			RamseteCommand followEndEnemyTrench = new RamseteCommand(
					paths.Defensive1()[4],
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

			return new SequentialCommandGroup(followToTrench, followThroughTrench, followTrenchToMid, followThroughMid
					, followEndEnemyTrench);
		}
	}

	public static class Defensive2 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand followToTrench = new RamseteCommand(
					paths.Defensive2()[0],
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

			RamseteCommand followThroughTrench = new RamseteCommand(
					paths.Defensive2()[1],
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

			RamseteCommand followTrenchToScoring = new RamseteCommand(
					paths.Defensive2()[2],
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

			return new SequentialCommandGroup( followToTrench, followThroughTrench, followTrenchToScoring );
		}
	}

	public static class Defensive3 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand followInitToOM = new RamseteCommand(
					paths.Defensive3()[0],
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

			RamseteCommand followThroughMid = new RamseteCommand(
					paths.Defensive3()[1],
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

			RamseteCommand followOMToScoring = new RamseteCommand(
					paths.Defensive3()[2],
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

			return new SequentialCommandGroup( followInitToOM, followThroughMid, followOMToScoring );
		}
	}

	public static class Defensive4 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					paths.Defensive4()[0],
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

	public static class PsuedoDefensive1 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					paths.PsuedoDefensive1()[0],
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

	public static class PsuedoDefensive2 {
		public static SequentialCommandGroup fullAutoSequence() {
			RamseteCommand ramseteCommand = new RamseteCommand(
					paths.PsuedoDefensive2()[0],
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
