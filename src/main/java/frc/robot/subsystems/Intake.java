/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;



public class Intake extends SubsystemBase {
   
    //define variables 
  private final WPI_VictorSPX intakeMotor;
    
  //constructor
  public Intake() {
    
  //instantiate motor controllers
  intakeMotor = new WPI_VictorSPX(Constants.Intake.intakeMotor);

  
  //safety
  intakeMotor.setSafetyEnabled(false);

  }

  public void startEat(){
    intakeMotor.set(Constants.Intake.maxSpeed);
  }
  public void stopEat(){
    intakeMotor.set(0);   
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}