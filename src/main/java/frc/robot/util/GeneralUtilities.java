package frc.robot.util;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

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
	
	public static Boolean WarmUp()
	{
		String warmUP = "";
		Double fooBar = 1.1;
		Double fooBarSin = 0.0;
		Double fooBarCos = 0.0;

		for(int loopCtr = 0; loopCtr<5000; loopCtr++)
		{
			warmUP = Double.toString(fooBar);
			fooBar = fooBar + 1.1;
			fooBarSin = Math.cos(fooBar);
			fooBarCos = Math.sin(fooBar);
		}

		NetworkTableInstance.getDefault();

		return true;
	}

	//  LeftTurn_v3 => LeftTurn_v3_left.csv  ==> LeftTurn_v3.left.pf1.csv
	public static void copyPathFiles(String pathName)
	{
		String filePathName;
		Path source;
		Path target;

		// *** LEFT ***
		// get the source file path
		filePathName = Filesystem.getDeployDirectory() 
								+ "/" + "paths/" + "output/" + pathName + "_left.csv";
		source = Paths.get(filePathName);

		// get the target file path
		filePathName = Filesystem.getDeployDirectory() 
								+ "/" + "paths/" + "output/" + pathName + ".left.pf1.csv";
		target = Paths.get(filePathName);

		try {
			Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// *** RIGHT ***
		// get the source file path
		filePathName = Filesystem.getDeployDirectory() 
								+ "/" + "paths/" + "output/" + pathName + "_right.csv";
		source = Paths.get(filePathName);

		// get the target file path
		filePathName = Filesystem.getDeployDirectory() 
								+ "/" + "paths/" + "output/" + pathName + ".right.pf1.csv";
		target = Paths.get(filePathName);

		try {
			Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static double getRoundedFPGATime()
	{
		double fpgaTime = ((double)RobotController.getFPGATime()) / 1000.0;

		return roundDouble(fpgaTime, 1);
	}
}