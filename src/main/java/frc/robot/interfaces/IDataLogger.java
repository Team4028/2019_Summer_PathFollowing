/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.interfaces;

import frc.robot.entities.LogDataBE;

/**
 * Add your docs here.
 */
public interface IDataLogger {

    public void initLogging(String robotMode);

	public void LogData(LogDataBE logData);

	public void close();

	// ====== Support for markers in the log file ======
	// can by used to flag certain areas in the file
	public void setMarker(String markerName);

	public void clearMarker();
}