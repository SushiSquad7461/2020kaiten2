/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

    //  define variables
    private final WPI_TalonSRX hopperFast;
    private final WPI_TalonSRX hopperSlow;

    public Hopper() {
        // instantiate motors
        hopperFast = new WPI_TalonSRX(Constants.Hopper.FAST_ID);
        hopperSlow = new WPI_TalonSRX(Constants.Hopper.SLOW_ID);
    }

    public void startSpit() {
        // set motor speed
        hopperFast.set(Constants.Hopper.MAX_SPEED);
        hopperSlow.set(0.6 * (Constants.Hopper.MAX_SPEED));
    }

    public void endSpit() {
        // sets to zero if there is a spike, so no action
        hopperFast.set(0);
        hopperSlow.set(0);
    }

    @Override
    public void periodic() {
        //  This method will be called once per scheduler run
    }
}