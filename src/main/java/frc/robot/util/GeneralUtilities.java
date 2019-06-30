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
			buildMsg = "===> Code Base: " + robotName + " | JAR DT: " + newDateString + " <===";
			DriverStation.reportWarning(buildMsg, false);
		} catch (URISyntaxException e) {
    		DriverStation.reportWarning("Error determining filename of current JAR file", true);
		} catch (IOException e) {	
    		DriverStation.reportWarning("General Error trying to determine current JAR file", true);
		}
		return buildMsg;
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
	 
	public static boolean isStringNullOrEmpty(String input){
        if(input == null || input.length() == 0){
            return true;
        }
        return false;
    }
}