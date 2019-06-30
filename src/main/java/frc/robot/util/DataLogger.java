/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.entities.LogDataBE;
import frc.robot.interfaces.IDataLogger;
import frc.robot.interfaces.LogDestination;

/**
 * In this version logging is sync to the control loops
 * 		(so time it takes for the write to complete can affect loop times)
 * - each write flushs to the storage device.
 */
public class DataLogger implements IDataLogger {

	// working variables
	private String _robotMode = "";
	private long _logStartTimeStampinMS;
	private long _lastScanTimeStampinMS;
	private long _markerStartTimeStampinMS;
	private String _markerName = null;

	private boolean _isLoggingEnabled;
	private boolean _isHeadersWrittenAlready;
	private String _logFolderPath;
	private PrintWriter _writer;
	private String _logFilePathName;

	// Constructor
	public DataLogger(LogDestination logDestination) {
		_logFolderPath = LoggingUtilities.CheckLogDestination(logDestination);

		_isLoggingEnabled = !(GeneralUtilities.isStringNullOrEmpty(_logFolderPath));
	}

	// init loging
	@Override
	public void initLogging(String robotMode) {

		if (_isLoggingEnabled) {
			_robotMode = robotMode;
			_logStartTimeStampinMS = RobotController.getFPGATime() / 1000;

			// create the writer here
			_logFilePathName = LoggingUtilities.BuildLogFilePathName(_logFolderPath, _robotMode);
			_writer = LoggingUtilities.CreateLogWriter(_logFilePathName);
		}
	}

	// synchronously write a log record to the file
	@Override
	public void LogData(LogDataBE logData) {

		if (_isLoggingEnabled) {

			// capture the start timestamp if this is the 1st call
			if (_logStartTimeStampinMS == 0) {
				_logStartTimeStampinMS = logData.get_logDataTimeStampinMS();
			}

			// add starting timestamp to log record
			logData.set_logStartTimeStampinMS(_logStartTimeStampinMS);

			// add marker to log record (if might be empty)
			logData.set_marker(_markerName);

			// optionally write the header row
			if (!_isHeadersWrittenAlready) {
				WriteHeaderLine(logData);
				_isHeadersWrittenAlready = true;
			}

			// write the data row
			WriteDataLine(logData);
		}
	}

    // Write out a header (label) row to the file
    private void WriteHeaderLine(LogDataBE logData) {

		_writer.print("StartDeltaMS" + "\t"
			+ "LastScanDeltaMS" + "\t" 
			+ "Marker" + "\t" 
			+ "MarkerElapsedMS" + "\t" 
			+ logData.BuildTSVHeader());

        _writer.flush();
    }

	// write out a data row to the file
	private void WriteDataLine(LogDataBE dataToLog) {

		double startDeltaDiffInMSecs = dataToLog.get_logDataTimeStampinMS() - _logStartTimeStampinMS;
		String markerName = dataToLog.get_marker();
		long lastScanDeltaInMS = 0;

		// calc last scan delta if this is not the 1st record
		if (_lastScanTimeStampinMS  > 0) {
			lastScanDeltaInMS = dataToLog.get_logDataTimeStampinMS() - _lastScanTimeStampinMS;
		}

		// write out the current data
		if(!GeneralUtilities.isStringNullOrEmpty(markerName)) {

			// calc elapsed time (in mS) since last marker set
			double markerElapsedInMSecs = dataToLog.get_logDataTimeStampinMS()  - _markerStartTimeStampinMS;

			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaInMS + "\t" 
							+ markerName + "\t" 
							+ markerElapsedInMSecs + "\t" 
							+ dataToLog.BuildTSVData());
		} else {
			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaInMS + "\t" 
							+ "\t" 
							+ "\t" 
							+ dataToLog.BuildTSVData());
		}

		_writer.flush();

		// snapshot this log record's timestamp to use in calc in next scan
		_lastScanTimeStampinMS = dataToLog.get_logDataTimeStampinMS();
	}

	// called when telop or auton is disabled to flush any remaining events to disk
	@Override
	public void close() {
	}

	// ====== Support for markers in the log file ======
	// can by used to flag certain areas in the file
	@Override
	public void setMarker(String markerName)
	{
		_markerName = markerName;
		_markerStartTimeStampinMS = RobotController.getFPGATime() / 1000;
	}

	@Override
	public void clearMarker()
	{
		_markerName = null;
	}

	@Override
	public boolean get_isLoggingEnabled() {
		return _isLoggingEnabled;
	}
}