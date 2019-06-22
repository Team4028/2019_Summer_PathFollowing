/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.entities.LogDataBE;

/**
 * This will take one of the following routes:
 * 	- build queue of events
 * 		- write all events out in close method
 * 		- create a separate thread that runs at a lower frequency
 */
public class DataLogger {

	// working variables
	private static String _robotMode = "";
	private static long _logStartTimeStampinMS;
	private String _markerName = null;
	private Queue<LogDataBE> _logEvents = new LinkedList<LogDataBE>();

	// ====== Open, write & close log files ======
	public static DataLogger setupLogging(String robotMode) {
		_robotMode = robotMode;
		_logStartTimeStampinMS = RobotController.getFPGATime() / 1000;

		return new DataLogger();
	}

	public void WriteDataLine(LogDataBE logData) {
		// add marker to log record
		logData.set_marker(_markerName);
		// add starting timestamp to log record
		logData.set_logStartTimeStampinMS(_logStartTimeStampinMS);

		// add log event to queue
		_logEvents.add(logData);
	}

	public void close() {
		// flush collected information to disk
		
		System.out.println("Total Log Records: " + _logEvents.size());
	}

	// ====== Support for markers in the log file ======
	// can by used to flag certain areas in the file
	public void setMarker(String markerName)
	{
		_markerName = markerName;
	}

	public void clearMarker()
	{
		_markerName = null;
	}
}