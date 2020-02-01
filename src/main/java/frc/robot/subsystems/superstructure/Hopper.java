/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

    //  define variables
    private final TalonSRX hopperFast;
    private final TalonSRX hopperSlow;

    public Hopper() {
        //  insatiate motors
        hopperFast = new TalonSRX(Constants.Hopper.FAST_ID);
        hopperSlow = new TalonSRX(Constants.Hopper.SLOW_ID);
    }
    public void startSpit() {

        //  config the peak and the minimum outputs to tell if there was a spike
        // [-1,1] represents [-100%, 100%]
        hopperFast.configNominalOutputForward(0, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configNominalOutputReverse(0, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configPeakOutputForward(1, Constants.Hopper.CONFIG_TIMEOUT);
        hopperFast.configPeakOutputReverse(-1, Constants.Hopper.CONFIG_TIMEOUT);

        //  sets the same configs to hopperSlow
        hopperSlow.follow(hopperFast);

        //  set motor speed
        hopperFast.set(Constants.Hopper.MAX_SPEED);
        hopperSlow.set(0.6 * (Constants.Hopper.MAX_SPEED));

        //  current spike
        if (hopperFast.OutputForward == 1 || hopperSlow.OutputForward == 1) {
            hopperFast.set(0);
            hopperSlow.set(0);
        }
    }
    public void endSpit() {
        //  sets to zero if there is a spike, so no action
        hopperFast.set(0);
        hopperSlow.set(0);
    }
    @Override
    public void periodic() {
        //  This method will be called once per scheduler run
    }
}