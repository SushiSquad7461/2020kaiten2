/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ClimbConstants;

public class Climb // extends ProfiledPIDSubsystem
	{

	// initialize the motors
	// private final WPI_TalonSRX deployTalon;
	private final WPI_TalonSRX winchTalon;
	private final WPI_VictorSPX winchVictor;
	private final CANCoder climbArmEncoder;
	private final ElevatorFeedforward climbArmFeedForward;

	// constructor
	public Climb()  {
		/*
		super(new ProfiledPIDController(ClimbConstants.ARM_kP, ClimbConstants.ARM_kI, ClimbConstants.ARM_kD,
				new TrapezoidProfile.Constraints(ClimbConstants.MAX_VELOCITY_RAD_PER_SEC, ClimbConstants.MAX_ACCEL))
				, ClimbConstants.BASE_POSE); */
		// define the motor controllers
		// deployTalon = new WPI_TalonSRX(ClimbConstants.DEPLOY_TALON);
		winchTalon = new WPI_TalonSRX(ClimbConstants.WINCH_TALON);
		winchVictor = new WPI_VictorSPX(ClimbConstants.WINCH_VICTOR);

		// set victors to follow talonsrx
		winchVictor.follow(winchTalon);

		// configure motors
		winchTalon.setInverted(ClimbConstants.TALON_INVERTED);
		winchVictor.setInverted(ClimbConstants.VICTOR_INVERTED);

		// initialize encoder
		climbArmEncoder = new CANCoder(ClimbConstants.CLIMB_CAN_ID);

		// initialize feedforward
		climbArmFeedForward = new ElevatorFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV, ClimbConstants.kA);
	}

	/* ProfiledPIDController is gone
	public void calculateInput(double input) {
		double output = (input/ClimbConstants.CLIMB_ELEVATOR_DISTANCE_PER_ROTATION);
		goToSetpoint(output);
	} */

	// theoretically the way to apply the ProfiledPIDController
		/* there is no more ProfiledPIDController
	public void goToSetpoint(double goalPose) {
		double calculateChange = m_controller.calculate(climbArmEncoder.getAbsolutePosition(), goalPose);
		double climbFF = climbArmFeedForward.calculate(m_controller.getSetpoint().position,
				m_controller.getSetpoint().velocity);
		deployTalon.setVoltage(calculateChange + climbFF);
	} */

	 // @Override
	public void useOutput(double output, TrapezoidProfile.State setpoint) { }

	// @Override
	public double getMeasurement() {
		return climbArmEncoder.getAbsolutePosition() + ClimbConstants.BASE_POSE;
	}

	// public void dropElevator() { goToSetpoint(ClimbConstants.BASE_POSE); }

		/*
	public void startDeployClimbArm() {
		deployTalon.set(ClimbConstants.DEPLOY_SPEED);
	}

	public void stopDeployClimbArm() {
		deployTalon.set(0);
	} */

	public void startLift() {
		winchTalon.set(ClimbConstants.LIFT_SPEED);
	}

	public void stopLift() {
		winchTalon.set(0);
	}

	public void dropLift() {
		winchTalon.set(ClimbConstants.WINCH_SPEED);
	}

	// @Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
