/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Chassis;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

	// define variables
	private CANSparkMax frontLeft, frontRight, backLeft, backRight;
	private Encoder leftEncoder, rightEncoder;
	//private AHRS nav;

	private boolean driveInverted;
	private DifferentialDrive differentialDrive;
	private DifferentialDriveKinematics driveKinematics;
	private DifferentialDriveOdometry driveOdometry;
	private SimpleMotorFeedforward leftFeedforward, rightFeedforward;
	private PIDController leftController, rightController;

	public Drivetrain() {

		// configuration
		CANSparkMaxLowLevel.MotorType brushless = CANSparkMaxLowLevel.MotorType.kBrushless;
		driveInverted = false;

		double wheelRadius = Constants.Drivetrain.wheelRadius;
		double encoderResolution = Constants.Drivetrain.encoderResolution;

		// instantiate objects
		frontLeft = new CANSparkMax(Constants.Drivetrain.FL_ID, brushless);
		frontRight = new CANSparkMax(Constants.Drivetrain.FR_ID, brushless);
		backLeft = new CANSparkMax(Constants.Drivetrain.BL_ID, brushless);
		backRight = new CANSparkMax(Constants.Drivetrain.BR_ID, brushless);

		frontLeft.restoreFactoryDefaults();
		frontRight.restoreFactoryDefaults();
		backLeft.restoreFactoryDefaults();
		backRight.restoreFactoryDefaults();

		leftEncoder = new Encoder(Constants.Drivetrain.ENCODER_LEFT_A, Constants.Drivetrain.ENCODER_LEFT_B);
		rightEncoder = new Encoder(Constants.Drivetrain.ENCODER_RIGHT_A, Constants.Drivetrain.ENCODER_RIGHT_B);

		leftEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / encoderResolution);
		rightEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / encoderResolution);

		//nav = new AHRS(SPI.Port.kMXP);
		//nav.reset();

		differentialDrive = new DifferentialDrive(frontLeft, frontRight);
		/*driveKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.trackWidth);
		driveOdometry = new DifferentialDriveOdometry(getAngle());*/

		leftFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.LEFT_kS, Constants.Drivetrain.LEFT_kV, Constants.Drivetrain.LEFT_kA);
		rightFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.RIGHT_kS, Constants.Drivetrain.RIGHT_kV, Constants.Drivetrain.RIGHT_kA);

		leftController = new PIDController(Constants.Drivetrain.LEFT_kP, Constants.Drivetrain.LEFT_kI, Constants.Drivetrain.LEFT_kD);
		rightController = new PIDController(Constants.Drivetrain.RIGHT_kP, Constants.Drivetrain.RIGHT_kI, Constants.Drivetrain.RIGHT_kD);

		// configure motor controllers
		backLeft.follow(frontLeft);
		backRight.follow(frontRight);

		frontLeft.setInverted(driveInverted);
		frontRight.setInverted(driveInverted);
		backLeft.setInverted(driveInverted);
		backRight.setInverted(driveInverted);

		frontLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		frontRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		backLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
		backRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);

	}

	// open loop curve drive method
	public void curveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {
		differentialDrive.curvatureDrive(linearVelocity, angularVelocity, isQuickTurn);
	}

	// closed loop drive method
	/*public void closedCurveDrive(double linearVelocity, double angularVelocity, boolean isQuickTurn) {

		ChassisSpeeds chassisSpeeds;
		DifferentialDriveWheelSpeeds wheelSpeeds;
		double leftOutput, rightOutput;
		double leftFeedforwardOutput, rightFeedforwardOutput;

		if (!isQuickTurn) {
			chassisSpeeds = new ChassisSpeeds(linearVelocity, 0, angularVelocity);
			wheelSpeeds = driveKinematics.toWheelSpeeds(chassisSpeeds);

			leftFeedforwardOutput = leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
			rightFeedforwardOutput = rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);

			leftOutput = leftController.calculate(leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
			rightOutput = rightController.calculate(rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

			frontLeft.set(leftOutput + leftFeedforwardOutput);
			frontRight.set(rightOutput + rightFeedforwardOutput);
		} else {
			leftOutput = angularVelocity;
			rightOutput = -angularVelocity;

			frontLeft.set(leftOutput);
			frontRight.set(rightOutput);
		}

	}*/

	// get angle from gyro
	public Rotation2d getAngle() {
		return Rotation2d.fromDegrees(35);
	}

	@Override
	public void periodic() {

	}

}
