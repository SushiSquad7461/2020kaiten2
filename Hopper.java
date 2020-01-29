/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

    //  define variables
    private final TalonSRX hopperFast;
    private final TalonSRX hopperSlow;

    public Hopper() {
        //  insatiate motors
        hopperFast = new TalonSRX(Constants.Hopper.hopperFast);
        hopperSlow = new TalonSRX(Constants.Hopper.hopperSlow);
    }
    public void startSpit() {
        //  set motor speed
        hopperFast.set(Constants.Hopper.maxSpeed);
        hopperSlow.set(0.6 * (Constants.Hopper.maxSpeed));
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