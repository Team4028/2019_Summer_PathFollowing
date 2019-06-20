package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.RobotController;

// implements logic to log data to text file
public class DataLogger {
	private PrintWriter _writer;
    
    private String _logFilePathName;
    private boolean _isLoggingEnabled;
	private boolean _isHeadersWrittenAlready;
	private long _loggingStartedDTMS;
	private long _lastScanDTMS;

	private String _markerName = null;
	private long _markerStartDTMS;

    // constructor, open a new timestamped log file in the target directory
	public DataLogger(String parentFolder, String fileSuffix) throws IOException 
	{
    	SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
		outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
		String newDateString = outputFormatter.format(new Date());
    	
    	// build the new filename
		String fileName = newDateString + "_" + fileSuffix + ".tsv";
		
    	// build the full file path name
    	_logFilePathName = parentFolder + File.separator + fileName;
    	
        _writer = new PrintWriter(new BufferedWriter(new FileWriter(_logFilePathName, true)));
    }

	public void setMarker(String markerName)
	{
		_markerName = markerName;
		//_markerStartDTMS = System.currentTimeMillis();
		_markerStartDTMS = RobotController.getFPGATime() / 1000;
	}

	public void clearMarker()
	{
		_markerName = null;
	}

    // Write a string to the file
    public void WriteHeaderLine(String textToLog) {              
		_writer.print("StartDeltaMS" + "\t"
					 + "LastScanDeltaMS" + "\t" 
					 + "Marker" + "\t" 
					 + "MarkerElapsedMS" + "\t" 
					 + textToLog);
        _writer.flush();
        
    	// init these values
		//_loggingStartedDTMS = System.currentTimeMillis();
		//_loggingStartedDTMS = RobotController.getFPGATime() / 1000;
    }

    // Write a structured data object to the log file
	public void WriteDataLine(LogDataBE dataToLog) 
	{
		// optionally write log header if this is 1st loop
		if(!_isHeadersWrittenAlready) {
			WriteHeaderLine(dataToLog.BuildTSVHeader());
			_loggingStartedDTMS = dataToLog.get_currentTimeInMS();
			_isHeadersWrittenAlready = true;
		}
		
		// calc elapsed time (in mS) since 1st scan
		//long startDeltaDiffInMSecs = System.currentTimeMillis() - _loggingStartedDTMS;
		//double startDeltaDiffInMSecs = (RobotController.getFPGATime() / 1000) - _loggingStartedDTMS;
		double startDeltaDiffInMSecs = dataToLog.get_currentTimeInMS() - _loggingStartedDTMS;

		// calc elapsed time (in mS) since last scan (should be 20 mS)
		double lastScanDeltaDiffInMS = 0;
		if (_lastScanDTMS  > 0)
		{
			//lastScanDeltaDiffInMS = System.currentTimeMillis() - _lastScanDTMS;
			//lastScanDeltaDiffInMS = (RobotController.getFPGATime() / 1000) - _lastScanDTMS;
			lastScanDeltaDiffInMS = dataToLog.get_currentTimeInMS() - _lastScanDTMS;
		}
		
		// write out the current data
		if(_markerName != null)
		{
			// calc elapsed time (in mS) since last marker set
			//long markerDeltaDiffInMSecs = System.currentTimeMillis() - _markerStartDTMS;
			//double markerElapsedInMSecs = (RobotController.getFPGATime() / 1000) - _markerStartDTMS;
			double markerElapsedInMSecs = dataToLog.get_currentTimeInMS()  - _markerStartDTMS;

			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaDiffInMS + "\t" 
							+ _markerName + "\t" 
							+ markerElapsedInMSecs + "\t" 
							+ dataToLog.BuildTSVData());
		}
		else
		{
			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaDiffInMS + "\t" 
							+ "\t" 
							+ "\t" 
							+ dataToLog.BuildTSVData());
		}

		//_writer.flush();
		
		// save last scan dt so we can calc delta on next scan
		//_lastScanDTMS = System.currentTimeMillis();
		//_lastScanDTMS = RobotController.getFPGATime() / 1000;
		_lastScanDTMS = dataToLog.get_currentTimeInMS();
    }
        
    public void close() {
		_writer.flush();
    	_writer.close(); // close the file
	}
	   
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	public boolean IsLoggingEnabled() {
		return _isLoggingEnabled;
	}
	
	public String getLogFilePathName() {
		return _logFilePathName;
	}
}