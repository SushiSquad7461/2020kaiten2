package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

public class Paths {
	private final RobotContainer m_container;
	private TrajectoryConfig config;
	public Trajectory toTrench, throughTrench, trenchToMid, throughMid, endMyTrench, trenchToScoring,
	initToOM, OMToScoring, initToScoring, scoringToMT, throughMTrench, MTToScoring, scoringToMM,
	throughMMToScoring, initLineThroughMM, initLineThroughMT, scoringToOT, scoringToOM,
	example = null;
	public Paths(RobotContainer container, DifferentialDriveVoltageConstraint voltageConstraint) {
		m_container = container;
		//RamseteCommands ramsete = new RamseteCommands(container);
		// configures trajectories
		config =
				new TrajectoryConfig(Constants.RamseteConstants.MAX_METERS_PER_SECOND,
						Constants.RamseteConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(container.s_drive.driveKinematics)
						// Apply the voltage constraint
						.addConstraint(voltageConstraint);

		example = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						//new Translation2d(1, 1),
						//new Translation2d(2, -1)
				),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				// Pass config
				config
		);

		String toTrenchJSON = "PathWeaver/Paths/to trench.wpilib.json";
		String throughTrenchJSON = "PathWeaver/Paths/through trench.wpilib.json";
		String trenchToMidJSON = "PathWeaver/Paths/to mid.wpilib.json";
		String throughMidJSON = "PathWeaver/Paths/through mid.wpilib.json";
		String endMyTrenchJSON = "PathWeaver/Paths/end my trench.wpilib.json";
		String trenchToScoringJSON = "PathWeaver/Paths/trench to scoring.wpilib.json";
		String initToOMJSON = "PathWeaver/Paths/init to oM.wpilib.json";
		String OMToScoringJSON = "PathWeaver/Paths/oM to scoring.wpilib.json";
		String initToScoringJSON = "PathWeaver/Paths/init to scoring.wpilib.json";
		String scoringToMTJSON = "PathWeaver/Paths/scoring to trench.wpilib.json";
		String throughMTTrenchJSON = "PathWeaver/Paths/scoring to trench.wpilib.json";
		String MTToScoringJSON = "PathWeaver/Paths/own trench to scoring.wpilib.json";
		String scoringToMMJSON = "PathWeaver/Paths/scoring to own mid.wpilib.json";
		String throughMMToScoringJSON = "PathWeaver/Paths/through own mid to scoring.wpilib.json";
		String initLineThroughMMidJSON = "PathWeaver/Paths/init line through mid.wpilib.json";
		String initLineThroughMTJSON = "PathWeaver/Paths/init line through my trench.wpilib.json";
		String scoringToOTJSON = "PathWeaver/Paths/scoring to other trench.wpilib.json";
		String scoringToOMJSON = "PathWeaver/Paths/scoring to other mid.wpilib.json";

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(toTrenchJSON);
			toTrench = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(throughTrenchJSON);
			throughTrench = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trenchToMidJSON);
			trenchToMid = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(throughMidJSON);
			throughMid = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(endMyTrenchJSON);
			endMyTrench = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trenchToScoringJSON);
			trenchToScoring = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(initToOMJSON);
			initToOM = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(OMToScoringJSON);
			OMToScoring = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(initToScoringJSON);
			initToScoring = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(scoringToMTJSON);
			scoringToMT = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(throughMTTrenchJSON);
			throughMTrench = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(MTToScoringJSON);
			MTToScoring = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(scoringToMMJSON);
			scoringToMM = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(throughMMToScoringJSON);
			throughMMToScoring = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(initLineThroughMMidJSON);
			initLineThroughMM = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(initLineThroughMTJSON);
			initLineThroughMT = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(scoringToOTJSON);
			scoringToOT = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(scoringToOMJSON);
			scoringToOM = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) { System.err.println(ex.getMessage());ex.printStackTrace();
		System.out.println("errorow"); }
	}
}
