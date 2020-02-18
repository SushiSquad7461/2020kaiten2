/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
   this will be re-written for the 4th time I'm calling it right now
*/

public class Flywheel extends PIDSubsystem {

	// define variables
	private final WPI_TalonSRX flywheelMain;
	private final WPI_VictorSPX flywheelSecondary;
	private final SimpleMotorFeedforward flywheelFeedforward;
	private CANCoder encoderMain;

	public Flywheel() {
		super(new PIDController(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD));

		// instantiate motors
		flywheelMain = new WPI_TalonSRX(Constants.Flywheel.MAIN_ID);
		flywheelSecondary = new WPI_VictorSPX(Constants.Flywheel.SECONDARY_ID);

		flywheelMain.configFactoryDefault();

		flywheelMain.setInverted(Constants.Flywheel.MAIN_INVERTED);
		flywheelSecondary.setInverted(Constants.Flywheel.SECONDARY_INVERTED);

		flywheelFeedforward = new SimpleMotorFeedforward(
				Constants.Flywheel.kS,
				Constants.Flywheel.kV,
				Constants.Flywheel.kA
		);

		// config the peak and nominal outputs ([-1, 1] represents [-100, 100]%)
		flywheelMain.configNominalOutputForward(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configNominalOutputReverse(0, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputForward(1, Constants.Flywheel.CONFIG_TIMEOUT);
		flywheelMain.configPeakOutputReverse(-1, Constants.Flywheel.CONFIG_TIMEOUT);

		flywheelSecondary.follow(flywheelMain);

		// encoder takes 2 ports
		encoderMain = new CANCoder(
				Constants.Flywheel.ENCODER_A
		);
	}

	public void stop() {
		flywheelMain.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void periodic() {
		// the first number here is a 0 for position tolerance, we want
		// it to be zero
		this.getController().setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
		this.getController().setSetpoint(Constants.Flywheel.SPEED);

		SmartDashboard.putNumber("flywheel rpm", encoderMain.getVelocity());
		SmartDashboard.putNumber("flywheel rpm 2", encoderMain.getVelocity());

		if (encoderMain.getVelocity() > Constants.Flywheel.SPEED - 100) {
			SmartDashboard.putBoolean("flywheel at speed", true);
		} else {
			SmartDashboard.putBoolean("flywheel at speed", false);
		}

		//RobotContainer.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, Math.pow(encoderMain.getVelocity() / 12000, 3));
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		//flywheelMain.set(ControlMode.PercentOutput, output + flywheelFeedforward.calculate(setpoint));
	}

	public void enableController() {
		double output = m_controller.calculate(encoderMain.getVelocity(), Constants.Flywheel.SPEED);
		double feedForward = flywheelFeedforward.calculate(Constants.Flywheel.SPEED);

		flywheelMain.setVoltage(output + feedForward);

		SmartDashboard.putNumber("controller output", output + feedForward);
	}

	@Override
	protected double getMeasurement() {
		return encoderMain.getVelocity();
	}

	public boolean isAtSpeed() {
		if (encoderMain.getVelocity() >= Constants.Flywheel.SPEED - 25) {
			return true;
		} else {
			return false;
		}
	}

}
