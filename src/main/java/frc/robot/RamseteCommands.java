package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RamseteCommands {
	private DifferentialDriveVoltageConstraint voltageConstraint;
	private static RamseteCommand exampleCommand,
	toTrench,
	throughTrench,
	trenchToMid,
	throughMid,
	midToMyTrench,
	trenchToScoring,
	initToOM,
	OMToScoring,
	initToScoring,
	scoringToMT,
	throughMT,
	MTToScoring,
	scoringToMM,
	throughMMToScoring,
	initLineThroughMM,
	initLineThroughMT;

	public RamseteCommands() {
		// sets voltage constraint so you dont over accelerate
		voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.RamseteConstants.kS,
						Constants.RamseteConstants.kV, Constants.RamseteConstants.kA), RobotContainer.s_drive.driveKinematics,
				Constants.RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

		exampleCommand = defineRamseteCommand(Paths.example);

		toTrench = defineRamseteCommand(Paths.toTrench);

		throughTrench = defineRamseteCommand(Paths.throughTrench);

		trenchToMid = defineRamseteCommand(Paths.trenchToMid);

		throughMid = defineRamseteCommand(Paths.throughMid);

		midToMyTrench = defineRamseteCommand(Paths.endMyTrench);

		trenchToScoring = defineRamseteCommand(Paths.trenchToScoring);

		initToOM = defineRamseteCommand(Paths.initToOM);

		OMToScoring = defineRamseteCommand(Paths.OMToScoring);

		initToScoring = defineRamseteCommand(Paths.initToScoring);

		scoringToMT = defineRamseteCommand(Paths.scoringToMT);

		throughMT = defineRamseteCommand(Paths.throughMTrench);

		MTToScoring = defineRamseteCommand(Paths.MTToScoring);

		scoringToMM = defineRamseteCommand(Paths.scoringToMM);

		throughMMToScoring = defineRamseteCommand(Paths.throughMMToScoring);

		initLineThroughMM = defineRamseteCommand(Paths.initLineThroughMM);

		initLineThroughMT = defineRamseteCommand(Paths.initLineThroughMT);

	}

	public RamseteCommand defineRamseteCommand(Trajectory traj) {

		return new RamseteCommand(
				traj,
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
