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
	private final TalonSRX flywheelMain;
	private final VictorSPX flywheelSecondary;
	private final SimpleMotorFeedforward flywheelFeedforward;
	private Encoder encoderMain;

	public Flywheel() {
		super(new PIDController(Constants.Flywheel.kP, Constants.Flywheel.kI, Constants.Flywheel.kD));

		// instantiate motors
		flywheelMain = new TalonSRX(Constants.Flywheel.MAIN_ID);
		flywheelSecondary = new VictorSPX(Constants.Flywheel.SECONDARY_ID);

		flywheelMain.configFactoryDefault();

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
		encoderMain = new Encoder(
				Constants.Flywheel.ENCODER_A,
				Constants.Flywheel.ENCODER_B,
				Constants.Flywheel.ENCODER_REVERSE_DIRECTION
		);
	}

	public void stop() {
		flywheelMain.set(ControlMode.Velocity, 0);
	}

	@Override
	public void periodic() {
		// the first number here is a 0 for position tolerance, we want
		// it to be zero
		this.getController().setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
		this.setSetpoint(Constants.Flywheel.SPEED);

		RobotContainer.operatorController.setRumble(GenericHID.RumbleType.kRightRumble, Math.pow(encoderMain.getRate(), 3));
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		flywheelMain.set(ControlMode.PercentOutput, output + flywheelFeedforward.calculate(setpoint));
	}

	@Override
	protected double getMeasurement() {
		return encoderMain.getRate();
	}

}
