/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.cuforge.libcu.Lasershark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LIDAR extends SubsystemBase {

	private static Lasershark lasershark;

	public LIDAR() {
		lasershark = new Lasershark(Constants.Lasershark.DIO);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("lasershark inches", lasershark.getDistanceInches());
	}
}
