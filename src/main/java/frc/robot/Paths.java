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
	private static TrajectoryConfig config;
	private static Trajectory toTrench = null;
	private static Trajectory throughTrench = null;
	private static Trajectory trenchToMid = null;
	private static Trajectory throughMid = null;
	private static Trajectory endMyTrench = null;
	private static Trajectory trenchToScoring = null;
	private static Trajectory initToOM = null;
	private static Trajectory OMToScoring = null;

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

	public static Trajectory[] Example() {
		// creates a new trajectory
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
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
		Trajectory[] finalPath = { exampleTrajectory };
		return finalPath;
	}

	public static Trajectory[] Offensive1() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] Offensive2() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] Defensive1() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { toTrench, throughTrench, trenchToMid, throughMid, endMyTrench };
		return finalPath;
	}

	public static Trajectory[] Defensive2() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { toTrench, throughTrench, trenchToScoring };
		return finalPath;
	}

	public static Trajectory[] Defensive3() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { initToOM, throughMid, OMToScoring };
		return finalPath;
	}

	public static Trajectory[] Defensive4() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] PsuedoDefensive1() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] PsuedoDefensive2() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

}
