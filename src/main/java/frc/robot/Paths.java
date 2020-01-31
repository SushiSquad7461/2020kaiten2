package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.List;

public class Paths {
	private static RamseteCommands ramsete;
	private static TrajectoryConfig config;

	public Paths() {
		ramsete = new RamseteCommands();
		// configures trajectories
		config =
				new TrajectoryConfig(Constants.RamseteConstants.MAX_METERS_PER_SECOND,
						Constants.RamseteConstants.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(RobotContainer.s_drive.driveKinematics)
						// Apply the voltage constraint
						.addConstraint(ramsete.getVoltageConstraint());
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

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] Defensive2() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
		return finalPath;
	}

	public static Trajectory[] Defensive3() {

		// insert a trajectory here (demonstrated in below webpage):
		// https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

		Trajectory[] finalPath = { };
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
