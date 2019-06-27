package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotMap;
import frc.robot.entities.LogDataBE;

// implements logic to log data to text file
// this is the original DataLogger class
public class DataLogger2 {
	private PrintWriter _writer;
    
    private String _logFilePathName;
    private boolean _isLoggingEnabled;
	private boolean _isHeadersWrittenAlready;
	private long _loggingStartedDTMS;
	private long _lastScanDTMS;

	private String _markerName = null;
	private long _markerStartDTMS;

    // constructor, open a new timestamped log file in the target directory
	public DataLogger2(String parentFolder, String fileSuffix) throws IOException {
    	SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
		outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
		String newDateString = outputFormatter.format(new Date());
    	
    	// build the new filename
		String fileName = newDateString + "_" + fileSuffix + ".tsv";
		
    	// build the full file path name
    	_logFilePathName = parentFolder + File.separator + fileName;
    	
        _writer = new PrintWriter(new BufferedWriter(new FileWriter(_logFilePathName, true)));
    }

	public void setMarker(String markerName) {
		_markerName = markerName;
		//_markerStartDTMS = System.currentTimeMillis();
		_markerStartDTMS = RobotController.getFPGATime() / 1000;
	}

	public void clearMarker() {
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
    }

    // Write a structured data object to the log file
	public void WriteDataLine(LogDataBE dataToLog) {
		// protect against timing issue where no data has been added yet
		if(dataToLog.get_isEmpty()) {
			return;
		}

		// optionally write log header if this is 1st loop
		if(!_isHeadersWrittenAlready) {
			WriteHeaderLine(dataToLog.BuildTSVHeader());
			_loggingStartedDTMS = dataToLog.get_logDataTimeStampinMS();
			_isHeadersWrittenAlready = true;
		}
		
		// calc elapsed time (in mS) since 1st scan
		//long startDeltaDiffInMSecs = System.currentTimeMillis() - _loggingStartedDTMS;
		//double startDeltaDiffInMSecs = (RobotController.getFPGATime() / 1000) - _loggingStartedDTMS;
		double startDeltaDiffInMSecs = dataToLog.get_logDataTimeStampinMS() - _loggingStartedDTMS;

		// calc elapsed time (in mS) since last scan (should be 20 mS)
		double lastScanDeltaDiffInMS = 0;
		if (_lastScanDTMS  > 0) {
			//lastScanDeltaDiffInMS = System.currentTimeMillis() - _lastScanDTMS;
			//lastScanDeltaDiffInMS = (RobotController.getFPGATime() / 1000) - _lastScanDTMS;
			lastScanDeltaDiffInMS = dataToLog.get_logDataTimeStampinMS() - _lastScanDTMS;
		}
		
		// write out the current data
		if(_markerName != null) {
			// calc elapsed time (in mS) since last marker set
			//long markerDeltaDiffInMSecs = System.currentTimeMillis() - _markerStartDTMS;
			//double markerElapsedInMSecs = (RobotController.getFPGATime() / 1000) - _markerStartDTMS;
			double markerElapsedInMSecs = dataToLog.get_logDataTimeStampinMS()  - _markerStartDTMS;

			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaDiffInMS + "\t" 
							+ _markerName + "\t" 
							+ markerElapsedInMSecs + "\t" 
							+ dataToLog.BuildTSVData());
		} else {
			_writer.print(startDeltaDiffInMSecs + "\t" 
							+ lastScanDeltaDiffInMS + "\t" 
							+ "\t" 
							+ "\t" 
							+ dataToLog.BuildTSVData());
		}


		_writer.flush();
		
		// save last scan dt so we can calc delta on next scan
		//_lastScanDTMS = System.currentTimeMillis();
		//_lastScanDTMS = RobotController.getFPGATime() / 1000;
		_lastScanDTMS = dataToLog.get_logDataTimeStampinMS();
    }
        
	public void close() {
		if(_writer != null)	{
			_writer.flush();
			_writer.close(); // close the file
		}
	}
	   
	//===============
	//
	//===============
	/** Optionally sets up logging if return object is null, logger is disabled */
	public static DataLogger2 setupLogging(String mode) {
		DataLogger2 dataLogger;
				
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.PRIMARY_LOG_FILE_PATH);
		Path alternatePath = Paths.get(RobotMap.ALTERNATE_LOG_FILE_PATH);
		if (Files.exists(path)) {
			try {
				dataLogger = new DataLogger2(RobotMap.PRIMARY_LOG_FILE_PATH, mode);
						
				System.out.println("-------------------------------------");
				System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
				System.out.println("-------------------------------------");
			} catch (IOException e) {
				e.printStackTrace();
				
				dataLogger = null;
				
				System.out.println("-------------------------------------");
				System.out.println("..Error configuring Logging to: " + RobotMap.PRIMARY_LOG_FILE_PATH);
				System.out.println("-------------------------------------");
			}
		}
		else if (Files.exists(alternatePath)) {
			try {
				dataLogger = new DataLogger2(RobotMap.ALTERNATE_LOG_FILE_PATH, mode);
								
				System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} catch (IOException e) {
				e.printStackTrace();
				
				dataLogger = null;
				
				System.out.println("..Error configuring Logging to: " + RobotMap.ALTERNATE_LOG_FILE_PATH);
			}
		} else {
			dataLogger = null;
			
			System.out.println("..Logging Disabled!");
		}
		return dataLogger;
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