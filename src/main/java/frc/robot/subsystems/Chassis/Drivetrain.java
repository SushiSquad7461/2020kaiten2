/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Chassis;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {

	// define variables
	private CANSparkMax frontLeft, frontRight, backLeft, backRight;

	private boolean driveInverted;
	private DifferentialDrive differentialDrive;

	public Drivetrain() {

		// configuration
		CANSparkMaxLowLevel.MotorType brushless = CANSparkMaxLowLevel.MotorType.kBrushless;
		driveInverted = false;

		// instantiate motor controllers
		frontLeft = new CANSparkMax(Constants.Drivetrain.FL_ID, brushless);
		frontRight = new CANSparkMax(Constants.Drivetrain.FR_ID, brushless);
		backLeft = new CANSparkMax(Constants.Drivetrain.BL_ID, brushless);
		backRight = new CANSparkMax(Constants.Drivetrain.BR_ID, brushless);

		differentialDrive = new DifferentialDrive(frontLeft, frontRight);

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

	@Override
	public void periodic() {

	}

}
