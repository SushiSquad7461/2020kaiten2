package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Shoot;

public class RamseteCommands {
	private final RobotContainer container;
	private final Paths path;
	private DifferentialDriveVoltageConstraint voltageConstraint;
	private RamseteCommand exampleCommand, toTrench, throughTrench, trenchToMid, throughMid, midToMyTrench,
	trenchToScoring, initToOM, OMToScoring, initToScoring, scoringToMT, throughMT, MTToScoring, scoringToMM,
	throughMMToScoring, initLineThroughMM, initLineThroughMT, scoringToOT, scoringToOM;

	public RamseteCommands() {
		container = new RobotContainer();
		path = new Paths();
		// sets voltage constraint so you dont over accelerate
		voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.RamseteConstants.kS,
						Constants.RamseteConstants.kV, Constants.RamseteConstants.kA), container.s_drive.driveKinematics,
				Constants.RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

		exampleCommand = defineRamseteCommand(path.example);

		toTrench = defineRamseteCommand(path.toTrench);

		throughTrench = defineRamseteCommand(path.throughTrench);

		trenchToMid = defineRamseteCommand(path.trenchToMid);

		throughMid = defineRamseteCommand(path.throughMid);

		midToMyTrench = defineRamseteCommand(path.endMyTrench);

		trenchToScoring = defineRamseteCommand(path.trenchToScoring);

		initToOM = defineRamseteCommand(path.initToOM);

		OMToScoring = defineRamseteCommand(path.OMToScoring);

		initToScoring = defineRamseteCommand(path.initToScoring);

		scoringToMT = defineRamseteCommand(path.scoringToMT);

		throughMT = defineRamseteCommand(path.throughMTrench);

		MTToScoring = defineRamseteCommand(path.MTToScoring);

		scoringToMM = defineRamseteCommand(path.scoringToMM);

		throughMMToScoring = defineRamseteCommand(path.throughMMToScoring);

		initLineThroughMM = defineRamseteCommand(path.initLineThroughMM);

		initLineThroughMT = defineRamseteCommand(path.initLineThroughMT);

		scoringToOT = defineRamseteCommand(path.scoringToOT);

		scoringToOM = defineRamseteCommand(path.scoringToOM);

	}

	public RamseteCommand defineRamseteCommand(Trajectory traj) {

		return new RamseteCommand(
				traj,
				container.s_drive::getPose,
				new RamseteController(Constants.RamseteConstants.RAMSETE_B, Constants.RamseteConstants.RAMSETE_ZETA),
				new SimpleMotorFeedforward(Constants.RamseteConstants.kS, Constants.RamseteConstants.kV, Constants.RamseteConstants.kA),
				container.s_drive.driveKinematics,
				container.s_drive::getWheelSpeeds,
				new PIDController(Constants.RamseteConstants.kP_VEL_LEFT, 0, 0),
				new PIDController(Constants.RamseteConstants.kP_VEL_RIGHT, 0, 0),
				// return the volts
				container.s_drive::tankDriveVolts,
				container.s_drive
		);
	}

	DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return voltageConstraint;
	}

	// this class is the entire path/sequence
	public SequentialCommandGroup ExampleAuto() {
		// returns the SequentialCommandGroup used in auto
			return new SequentialCommandGroup(exampleCommand);
	}

	public SequentialCommandGroup Offensive1() {
			return new SequentialCommandGroup(
					initToScoring, container.c_shoot.withTimeout(2), scoringToMT,
					new ParallelCommandGroup(throughMT, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					new RunCommand(container.s_intake::stopVore), MTToScoring, container.c_shoot);
	}

	public SequentialCommandGroup Offensive2() {
			return new SequentialCommandGroup( initToScoring, container.c_shoot.withTimeout(2), scoringToMM,
					new ParallelCommandGroup(throughMMToScoring, new RunCommand(container.s_intake::startVore)).withTimeout(3),
					new RunCommand(container.s_intake::stopVore), container.c_shoot);
	}

	public SequentialCommandGroup Defensive1() {
			return new SequentialCommandGroup( container.c_shoot.withTimeout(1.5), toTrench,
					new ParallelCommandGroup(throughTrench, new RunCommand(container.s_intake::startVore).withTimeout(3)),
					container.c_shoot.withTimeout(2),
					new ParallelCommandGroup(throughMid, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					midToMyTrench );
	}

	public SequentialCommandGroup Defensive2() {
			return new SequentialCommandGroup( container.c_shoot.withTimeout(2), toTrench,
					new ParallelCommandGroup(throughTrench, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					trenchToScoring, container.c_shoot );
	}

	public SequentialCommandGroup Defensive3() {
			return new SequentialCommandGroup( container.c_shoot.withTimeout(2), initToOM,
					new ParallelCommandGroup(throughMid, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					OMToScoring, container.c_shoot );
	}

	public SequentialCommandGroup CounterDefensive1() {
			return new SequentialCommandGroup( container.c_shoot.withTimeout(2), initLineThroughMM,
					throughMMToScoring, container.c_shoot );
	}

	public SequentialCommandGroup CounterDefensive2() {
			return new SequentialCommandGroup( container.c_shoot.withTimeout(2),
					new ParallelCommandGroup(initLineThroughMT, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					MTToScoring, container.c_shoot );
	}

	public SequentialCommandGroup PseudoOffensive1() {
			return new SequentialCommandGroup( initToScoring, container.c_shoot.withTimeout(2),
					scoringToOT, new ParallelCommandGroup(throughTrench, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					trenchToScoring, container.c_shoot );
	}

	public SequentialCommandGroup PseudoOffensive2() {
			return new SequentialCommandGroup( initToScoring, container.c_shoot.withTimeout(2), scoringToOM,
					new ParallelCommandGroup(throughMid, new RunCommand(container.s_intake::startVore).withTimeout(2)),
					OMToScoring, container.c_shoot );
	}
}
