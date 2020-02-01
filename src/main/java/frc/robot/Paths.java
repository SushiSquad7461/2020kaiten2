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

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

public class Paths {
	public static TrajectoryConfig config;
	public static Trajectory toTrench = null;
	public static Trajectory throughTrench = null;
	public static Trajectory trenchToMid = null;
	public static Trajectory throughMid = null;
	public static Trajectory endMyTrench = null;
	public static Trajectory trenchToScoring = null;
	public static Trajectory initToOM = null;
	public static Trajectory OMToScoring = null;
	public static Trajectory example = null;
	public Paths() {
		RamseteCommands ramsete = new RamseteCommands();
		// configures trajectories
		config =
				new TrajectoryConfig(Constants.RamseteConstants.MAX_METERS_PER_SECOND,
						Constants.RamseteConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(RobotContainer.s_drive.driveKinematics)
						// Apply the voltage constraint
						.addConstraint(ramsete.getVoltageConstraint());

		example = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(
						new Translation2d(1, 1),
						new Translation2d(2, -1)
				),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				// Pass config
				config
		);

		String toTrenchJSON = "PathWeaver/Paths/to trench.JSON";
		String throughTrenchJSON = "PathWeaver/Paths/through trench.JSON";
		String trenchToMidJSON = "PathWeaver/Paths/to mid.JSON";
		String throughMidJSON = "PathWeaver/Paths/through mid.JSON";
		String endMyTrenchJSON = "PathWeaver/Paths/end my trench.JSON";
		String trenchToScoringJSON = "PathWeaver/Paths/trench to scoring.JSON";
		String initToOMJSON = "PathWeaver/Paths/init to oM.JSON";
		String OMToScoringJSON = "PathWeaver/Paths/oM to scoring.JSON";

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
		} catch (IOException ex) {  }
	}
}
