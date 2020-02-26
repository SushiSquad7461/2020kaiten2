/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

	// define variables
	private final WPI_VictorSPX intakeMotor;
	private final WPI_TalonSRX intakeArm;
	private final ProfiledPIDController controller;
	private final CANCoder encoder;
	private final ArmFeedforward intakeFF;

	// constructor
	public Intake() {

		// instantiate motor controllers
		intakeMotor = new WPI_VictorSPX(Constants.Intake.MOTOR_ID);
		intakeArm = new WPI_TalonSRX(Constants.Intake.ARM_MOTOR_ID);
		encoder = new CANCoder(Constants.Intake.ENCODER_ID);

		// instantiate ProfiledPIDController
		intakeFF = new ArmFeedforward(Constants.Intake.kS, Constants.Intake.kV, Constants.Intake.kA);
		controller = new ProfiledPIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD,
				new TrapezoidProfile.Constraints(Constants.Intake.MAX_VEL, Constants.Intake.MAX_ACCEL));

		// configuration
		intakeMotor.setSafetyEnabled(false);
		intakeArm.setSafetyEnabled(false);

	}

	public void extendIntake() {
		double output = controller.calculate(encoder.getAbsolutePosition(), Constants.Intake.GOAL_POSE);
		double ff = intakeFF.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);
		intakeArm.setVoltage(output + ff);
	}

	public void retractIntake() {
		double output = controller.calculate(encoder.getAbsolutePosition(), Constants.Intake.START_POSE);
		double ff = intakeFF.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity);
		intakeArm.setVoltage(output + ff);
	}

	public void startVore(){
		intakeMotor.set(Constants.Intake.MAX_SPEED);
	}
	public void reverseVore() { intakeMotor.set(-Constants.Intake.MAX_SPEED); }
	public void stopVore(){
		intakeMotor.set(0);
	}

	@Override
	public void periodic() {

	}
}
