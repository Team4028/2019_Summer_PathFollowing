/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.io.PrintWriter;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.entities.LogDataBE;
import frc.robot.interfaces.IDataLogger;
import frc.robot.interfaces.LogDestination;

/**
 * This version tries to make logging totally async to the control loops
 * 
 * - build queue of events 
 * - create a separate thread that runs at a lower frequency to write to storage
 * - flush any remaining events out in close method
 */
public class DataLoggerV2 implements IDataLogger {

	// working variables
	private String _robotMode = "";
	private long _logStartTimeStampinMS;
	private long _lastScanTimeStampinMS;
	private long _markerStartTimeStampinMS;
	private String _markerName = null;

	private boolean _isLoggingEnabled;
	private boolean _isHeadersWrittenAlready;
	private Queue<LogDataBE> _logEventsBuffer = new ConcurrentLinkedQueue<LogDataBE>();
	private String _logFolderPath;
	private PrintWriter _writer;

	ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();

	// Constructor
	public DataLoggerV2(LogDestination logDestination) {
		_logFolderPath = LoggingUtilities.CheckLogDestination(logDestination);

		_isLoggingEnabled = !(GeneralUtilities.isStringEmpty(_logFolderPath));
	}

	// start log queue polling thread
	@Override
	public void initLogging(String robotMode) {

		if (_isLoggingEnabled) {
			_robotMode = robotMode;
			_logStartTimeStampinMS = RobotController.getFPGATime() / 1000;

			// setup runnable to asyc flush log records to the target location
			Runnable runnable = new Runnable() {
				public void run() {
					flushQueue();
				}
			};

			// run the thread every second
			//  we need to be carefull that the previous execution is not still running
			service.scheduleAtFixedRate(runnable, 0, 1, TimeUnit.SECONDS);
		}
	}

	// synchronously add a log record to the queue
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

			// add log event to queue
			synchronized (_logEventsBuffer) {
				_logEventsBuffer.add(logData);
			}
		}
	}

	// run asynchronously to flush log records to permanent storage
	private void flushQueue() {
		// working variable outside loop
		LogDataBE logData = null;

		// stay here and loop until we empty the buffer
		while (!_logEventsBuffer.isEmpty()) {
			// hold the lock for as short a time as possible
			synchronized (_logEventsBuffer) {
				// get the log record at the front of the queue
				logData = _logEventsBuffer.poll();
			}

			// optionally write the header row
			if (!_isHeadersWrittenAlready) {
				// we defer the creation of the writer to this background thread
				_writer = LoggingUtilities.CreateLogWriter(_logFolderPath, _robotMode);

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
			+ logData.BuildTSVHeader()
			+ "\t" + "LogQueueDepth");

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
		if(!GeneralUtilities.isStringEmpty(markerName)) {

			// calc elapsed time (in mS) since last marker set
			double markerElapsedInMSecs = dataToLog.get_logDataTimeStampinMS()  - _markerStartTimeStampinMS;

			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaInMS + "\t" 
							+ markerName + "\t" 
							+ markerElapsedInMSecs + "\t" 
							+ dataToLog.BuildTSVData()
							+ "\t" + _logEventsBuffer.size());
		} else {
			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaInMS + "\t" 
							+ "\t" 
							+ "\t" 
							+ dataToLog.BuildTSVData()
							+ "\t" + _logEventsBuffer.size());
		}

		_writer.flush();

		// snapshot this log record's timestamp to use in calc in next scan
		_lastScanTimeStampinMS = dataToLog.get_logDataTimeStampinMS();
	}

	// called when telop or auton is disabled to flush any remaining events to disk
	@Override
	public void close() {
		// stop the polling thread
		service.shutdown();

		// flush collected information to disk
		if(!_logEventsBuffer.isEmpty()) {
			flushQueue();
		}
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
}