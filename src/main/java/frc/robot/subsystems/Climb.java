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

public class Climb extends ProfiledPIDSubsystem {

	// initialize the motors
	private final WPI_TalonSRX climbTalon;
	private final WPI_TalonSRX climbTalonFollower;
	private final CANCoder climbArmEncoder;
	private final ElevatorFeedforward climbArmFeedForward;

	// constructor
	public Climb()  {
		super(new ProfiledPIDController(ClimbConstants.ARM_kP, ClimbConstants.ARM_kI, ClimbConstants.ARM_kD,
				new TrapezoidProfile.Constraints(ClimbConstants.MAX_VELOCITY_RAD_PER_SEC, ClimbConstants.MAX_ACCEL))
				, ClimbConstants.BASE_POSE);
		// define the motor controllers
		climbTalon = new WPI_TalonSRX(ClimbConstants.DEPLOY_TALON);
		climbTalonFollower = new WPI_TalonSRX(ClimbConstants.FOLLOWER_TALON);

		// configure motors
		climbTalon.configFactoryDefault();
		climbTalonFollower.configFactoryDefault();

		climbTalon.setSafetyEnabled(false);
		climbTalonFollower.setSafetyEnabled(false);

		climbTalon.setInverted(ClimbConstants.TALON_INVERTED);
		climbTalonFollower.setInverted(ClimbConstants.TALON_INVERTED);
		climbTalonFollower.follow(climbTalon);

		// initialize encoder
		climbArmEncoder = new CANCoder(ClimbConstants.CLIMB_CAN_ID);

		// initialize feedforward
		climbArmFeedForward = new ElevatorFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV, ClimbConstants.kA);
	}

	// theoretically the way to apply the ProfiledPIDController
	public void goToSetpoint(double goalPose) {
		double calculateChange = m_controller.calculate(climbArmEncoder.getAbsolutePosition(), goalPose);
		double climbFF = climbArmFeedForward.calculate(m_controller.getSetpoint().position,
				m_controller.getSetpoint().velocity);
		climbTalon.setVoltage(calculateChange + climbFF);
	}

	@Override
	public void useOutput(double output, TrapezoidProfile.State setpoint) { }

	@Override
	public double getMeasurement() {
		return climbArmEncoder.getAbsolutePosition() + ClimbConstants.BASE_POSE;
	}

	public void climbUp() { climbTalon.set(ClimbConstants.CLIMB_SPEED); }

	public void stopClimb() { climbTalon.set(0); }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
