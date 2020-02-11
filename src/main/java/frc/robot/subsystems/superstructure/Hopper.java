/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import javax.naming.ldap.Control;

public class Hopper extends SubsystemBase {

	//  define variables
	private final TalonSRX hopperFast;
	private final TalonSRX hopperSlow;

	public boolean currentSpike;

	public Hopper() {

		//  instantiate motors
		hopperFast = new TalonSRX(Constants.Hopper.FAST_ID);
		hopperSlow = new TalonSRX(Constants.Hopper.SLOW_ID);

		//  config the peak and the minimum outputs to tell if there was a spike
		// [-1,1] represents [-100%, 100%]
		hopperFast.configNominalOutputForward(0, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configNominalOutputReverse(0, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configPeakOutputForward(1, Constants.Hopper.CONFIG_TIMEOUT);
		hopperFast.configPeakOutputReverse(-1, Constants.Hopper.CONFIG_TIMEOUT);

		//  sets the same configs to hopperSlow
		hopperSlow.follow(hopperFast);

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
		hopperFast.set(ControlMode.PercentOutput, 0);
		hopperSlow.set(ControlMode.PercentOutput, 0);

	}

	public boolean isCurrentSpiked() {
		if (hopperFast.getSupplyCurrent() >= Constants.Hopper.CURRENT_SPIKE || hopperSlow.getSupplyCurrent() >= Constants.Hopper.CURRENT_SPIKE) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void periodic() {
		currentSpike = isCurrentSpiked();
	}
}