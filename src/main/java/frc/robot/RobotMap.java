/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// Drivers Station Gamepad USB Ports
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int ENGINEERING_GAMEPAD_USB_PORT = 2;
	
	// PCM CAN Bus Address
	public static final int PCM_CAN_ADDR = 0;
	
	// Motor Controller Can Bus Address
	public static final int LEFT_DRIVE_MASTER_CAN_ADDR = 11;
  public static final int LEFT_DRIVE_SLAVE1_CAN_ADDR = 14;
  public static final int LEFT_DRIVE_SLAVE2_CAN_ADDR = 15;
	public static final int RIGHT_DRIVE_MASTER_CAN_ADDR = 10;
	public static final int RIGHT_DRIVE_SLAVE1_CAN_ADDR = 12;
	public static final int RIGHT_DRIVE_SLAVE2_CAN_ADDR = 13;
  
  // NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  
	// You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
  public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
}
