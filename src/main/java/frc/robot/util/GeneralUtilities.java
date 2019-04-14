package frc.robot.util;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;

public class GeneralUtilities {		
    /** Writes general info about the build to the Operator's Console */
	public static String WriteBuildInfoToDashboard(String robotName) {
		String buildMsg = "?";
		try {
    		//get the path of the currently executing jar file
			String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();		
			Path filePath = Paths.get(currentJarFilePath);
			
			//get file system details from current file
			BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
			Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());
	
			// convert from UTC to local time zone
			SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
			outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
			String newDateString = outputFormatter.format(utcFileDate);
			
			// write the build date & time to the operator's console log window
			buildMsg = "== " + robotName + " | " + newDateString + " ==";
			DriverStation.reportWarning(buildMsg, false);
		} catch (URISyntaxException e) {
    		DriverStation.reportWarning("Error determining filename of current JAR file", true);
			//e.printStackTrace();
		} catch (IOException e) {	
    		DriverStation.reportWarning("General Error trying to determine current JAR file", true);
			//e.printStackTrace();
		}
		return buildMsg;
	}
	
    /** Optionally sets up logging if return object is null, logger is disabled */
	public static DataLogger setupLogging(String mode) {
		DataLogger dataLogger;
				
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.PRIMARY_LOG_FILE_PATH);
		Path alternatePath = Paths.get(RobotMap.ALTERNATE_LOG_FILE_PATH);
    	if (Files.exists(path)) {
    		try {
				dataLogger = new DataLogger(RobotMap.PRIMARY_LOG_FILE_PATH, mode);
					  
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
				dataLogger = new DataLogger(RobotMap.ALTERNATE_LOG_FILE_PATH, mode);
					    		
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
	
    // This method rounds a double to the specified # of decimal places
	public static double roundDouble(Double originalValue, int decimalPlaces) {
		BigDecimal bd = new BigDecimal(originalValue).setScale(decimalPlaces, RoundingMode.HALF_EVEN);
		
		return bd.doubleValue();
	}
    
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    
    /** This method makes sure a value is between a max & min value */
 	public static int ClampValue(int original, int min, int max) {
 		return Math.min(max, Math.max(min, original));
 	}
}