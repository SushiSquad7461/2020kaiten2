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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

/*
   this will be re-written for the 5th time I'm calling it right now
*/

public class Flywheel extends SubsystemBase {

  // define variables
  private final TalonSRX flywheelMain;
  private final VictorSPX flywheelSecondary;
  private final SimpleMotorFeedforward m_MotorFeedforward;
  private final PIDController pidController;
  private CANCoder encoderMain;
  private Mode currentMode = Mode.LINED_UP;

  // Used when calling enable() to set the mode, in case
  // we want to change how far the flywheel shoots.
  public enum Mode {
    LINED_UP (0.75), // When we're lined up with the power port
    PASSING (0.2), // When we would want to throw it a short distance to our alliance partner
    YEET (1); // If we want to shoot as far as possible, either for passing to an alliance partner or just for fun
  
    public final double speed;
    Mode(double speed) {
      this.speed = speed;
    }
  } 

  public Flywheel() {

    // instantiate motors
    flywheelMain = new TalonSRX(Constants.Flywheel.MAIN_ID);
    flywheelSecondary = new VictorSPX(Constants.Flywheel.SECONDARY_ID);

    flywheelMain.configFactoryDefault();

    m_MotorFeedforward = new SimpleMotorFeedforward(Constants.Flywheel.kS, Constants.Flywheel.kV,
        Constants.Flywheel.kA);

    pidController = new PIDController(
      Constants.Flywheel.kP,
      Constants.Flywheel.kI,
      Constants.Flywheel.kD);

    
    /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
    flywheelMain.configNominalOutputForward(0, Constants.Flywheel.CONFIG_TIMEOUT);
    flywheelMain.configNominalOutputReverse(0, Constants.Flywheel.CONFIG_TIMEOUT);
    flywheelMain.configPeakOutputForward(1, Constants.Flywheel.CONFIG_TIMEOUT);
    flywheelMain.configPeakOutputReverse(-1, Constants.Flywheel.CONFIG_TIMEOUT);

    flywheelSecondary.follow(flywheelMain);

    // encoder takes 2 ports
    encoderMain = new CANCoder(Constants.Flywheel.ENCODER);
  }




  @Override
  public void periodic() {
    // the first number here is a 0 for position tolerance, we want
    // it to be zero
    pidController.setTolerance(0, Constants.Flywheel.ERROR_TOLERANCE);
  }


  protected double getMeasurement() {
    return encoderMain.getVelocity();
  }

  public void enable(Mode mode) {
    this.currentMode = mode;
    double output = pidController.calculate(this.getMeasurement(), currentMode.speed);
    double feedForward = m_MotorFeedforward.calculate(currentMode.speed);
    flywheelMain.set(ControlMode.Velocity, output + feedForward);
  }

  public void disable() {
    flywheelMain.set(ControlMode.Velocity, 0);
  }

}
