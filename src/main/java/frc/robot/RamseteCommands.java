package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RamseteCommands {
	private DifferentialDriveVoltageConstraint voltageConstraint;
	private static RamseteCommand exampleCommand;
	private static RamseteCommand toTrench;
	private static RamseteCommand throughTrench;
	private static RamseteCommand trenchToMid;
	private static RamseteCommand throughMid;
	private static RamseteCommand midToMyTrench;
	private static RamseteCommand trenchToScoring;
	private static RamseteCommand initToOM;
	private static RamseteCommand OMToScoring;
	private static RamseteCommand initToScoring;
	private static RamseteCommand scoringToMT;
	private static RamseteCommand throughMT;
	private static RamseteCommand MTToScoring;
	private static RamseteCommand scoringToMM;
	private static RamseteCommand throughMMToScoring;
	private static RamseteCommand initLineThroughMM;
	private static RamseteCommand initLineThroughMT;

	public RamseteCommands() {
		// sets voltage constraint so you dont over accelerate
		voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.RamseteConstants.kS,
						Constants.RamseteConstants.kV, Constants.RamseteConstants.kA), RobotContainer.s_drive.driveKinematics,
				Constants.RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

		exampleCommand = new RamseteCommand(
				Paths.example,
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

		toTrench = new RamseteCommand(
				Paths.toTrench,
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

		throughTrench = new RamseteCommand(
				Paths.throughTrench,
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

		trenchToMid = new RamseteCommand(
				Paths.trenchToMid,
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

		throughMid = new RamseteCommand(
				Paths.throughMid,
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

		midToMyTrench = new RamseteCommand(
				Paths.endMyTrench,
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

		trenchToScoring = new RamseteCommand(
				Paths.trenchToScoring,
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

		initToOM = new RamseteCommand(
				Paths.initToOM,
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

		OMToScoring = new RamseteCommand(
				Paths.OMToScoring,
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

		initToScoring = new RamseteCommand(
				Paths.initToScoring,
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

		scoringToMT = new RamseteCommand(
				Paths.scoringToMT,
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

		throughMT = new RamseteCommand(
				Paths.throughMTrench,
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

		MTToScoring = new RamseteCommand(
				Paths.MTToScoring,
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

		scoringToMM = new RamseteCommand(
				Paths.scoringToMM,
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

		throughMMToScoring = new RamseteCommand(
				Paths.throughMMToScoring,
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

		initLineThroughMM = new RamseteCommand(
				Paths.initLineThroughMM,
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

		initLineThroughMT = new RamseteCommand(
				Paths.initLineThroughMT,
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

	DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return voltageConstraint;
	}

	// this class is the entire path/sequence
	static class ExamplePath {
		// returns the SequentialCommandGroup used in auto
		static SequentialCommandGroup fullAutoSequence() {

			return new SequentialCommandGroup(exampleCommand);
		}
	}

	static class Offensive1 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( initToScoring, scoringToMT, throughMT, MTToScoring );
		}
	}

	static class Offensive2 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( initToScoring, scoringToMM, throughMMToScoring );
		}
	}

	static class Defensive1 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( toTrench, throughTrench , trenchToMid , throughMid, midToMyTrench );
		}
	}

	static class Defensive2 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( toTrench, throughTrench, trenchToScoring );
		}
	}

	static class Defensive3 {
		 static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( initToOM, throughMid, OMToScoring );
		}
	}

	static class CounterDefensive1 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( initLineThroughMM, throughMMToScoring );
		}
	}

	static class CounterDefensive2 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( initLineThroughMT, MTToScoring );
		}
	}

	static class PseudoOffensive1 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( exampleCommand );
		}
	}

	static class PseudoOffensive2 {
		static SequentialCommandGroup fullAutoSequence() {
			return new SequentialCommandGroup( exampleCommand );
		}
	}

}
