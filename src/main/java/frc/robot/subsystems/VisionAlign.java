/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionAlign extends SubsystemBase {

	// create variables
	NetworkTableInstance table;
	NetworkTable cameraTable;
	NetworkTableEntry yaw;

	private double kP;

	// constructor
	public VisionAlign() {
		table = NetworkTableInstance.getDefault();
		cameraTable = table.getTable(Constants.Camera.NETWORK_TABLE).getSubTable(Constants.Camera.SUBTABLE1);
		yaw = cameraTable.getEntry("yaw");

		kP = Constants.Camera.TARGET_ALIGN_kP;
	}

	// find angular error
	public double calculateOutput() {
		if (Math.abs(yaw.getDouble(0)) > 2) {
			return -kP * yaw.getDouble(0);
		} else {
			return 0;
		}
	}

	// align the robot to the vision target
	public void alignRobot() {
		//RobotContainer.s_drive.curveDrive(); // open loop drive
		RobotContainer.s_drive.closedCurveDrive(Constants.Camera.TARGET_ALIGN_FORWARD, calculateOutput() * Constants.Camera.TARGET_ALIGN_ANGULAR, true); // closed loop drive
	}

	public void cancelAlign() {
		RobotContainer.s_drive.closedCurveDrive(0, 0, false);
	}

	@Override
	public void periodic() {

	}
}
