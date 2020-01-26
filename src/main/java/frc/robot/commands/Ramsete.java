/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Paths;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.Constants.RamseteConstants;

import java.util.List;

public class Ramsete extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
	public Drivetrain s_drivetrain;

	public Ramsete(Drivetrain subsystem) {
		s_drivetrain = subsystem;
	}

	// sets voltage constraint so you dont over accelerate
	public DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
			new SimpleMotorFeedforward(RamseteConstants.kS,
					RamseteConstants.kV, RamseteConstants.kA), s_drivetrain.driveKinematics,
			RamseteConstants.MAX_VOLTAGE_CONSTRAINT);

	public RamseteCommand ramseteCommand = new RamseteCommand(
			Paths.Example(),
			s_drivetrain::getPose,
			new RamseteController(RamseteConstants.RAMSETE_B, RamseteConstants.RAMSETE_ZETA),
			new SimpleMotorFeedforward(RamseteConstants.kS, RamseteConstants.kV, RamseteConstants.kA),
			s_drivetrain.driveKinematics,
			s_drivetrain::getWheelSpeeds,
			new PIDController(RamseteConstants.kP_VEL, 0, 0),
			new PIDController(RamseteConstants.kP_VEL, 0, 0),
				// return the volts
			s_drivetrain::tankDriveVolts,
			s_drivetrain
	);

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
