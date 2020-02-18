/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import javax.naming.ldap.Control;

public class Hopper extends SubsystemBase {

	//  define variables
	private final WPI_TalonSRX hopperFast;
	private final WPI_VictorSPX hopperSlow;

	public boolean currentSpike;

	public Hopper() {

		//  instantiate motors
		hopperFast = new WPI_TalonSRX(Constants.Hopper.FAST_ID);
		hopperSlow = new WPI_VictorSPX(Constants.Hopper.SLOW_ID);

		hopperFast.configFactoryDefault();
		hopperSlow.configFactoryDefault();

		//  config the peak and the minimum outputs to tell if there was a spike
		// [-1,1] represents [-100%, 100%]
		hopperFast.configNominalOutputForward(0, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configNominalOutputReverse(0, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configPeakOutputForward(1, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configPeakOutputReverse(-1, Constants.Hopper.CONFIG_TIMEOUT);

		currentSpike = false;

	}

	public void startSpit() {

		//  set motor speed
		hopperFast.set(ControlMode.PercentOutput, Constants.Hopper.MAX_SPEED);
		hopperSlow.set(ControlMode.PercentOutput, Constants.Hopper.SLOW_SPEED);

	}

	public void reverseSpit() {

		hopperFast.set(ControlMode.PercentOutput, Constants.Hopper.REVERSE_SPEED);
		hopperSlow.set(ControlMode.PercentOutput, Constants.Hopper.REVERSE_SPEED);

	}

	public void endSpit() {

		//  sets to zero
		hopperFast.set(ControlMode.PercentOutput, Constants.Hopper.STOP_SPEED);
		hopperSlow.set(ControlMode.PercentOutput, Constants.Hopper.STOP_SPEED);

	}

	public boolean isCurrentSpiked() {
		/*if (hopperFast.getSupplyCurrent() >= Constants.Hopper.CURRENT_SPIKE) {
			return true;
		} else {
			return false;
		}*/
		SmartDashboard.putBoolean("hopper spiked", (hopperFast.getSupplyCurrent() >= Constants.Hopper.CURRENT_SPIKE));
		return false;
	}

	@Override
	public void periodic() {
		currentSpike = isCurrentSpiked();
	}
}