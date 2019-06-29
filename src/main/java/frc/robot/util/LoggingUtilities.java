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
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import frc.robot.RobotMap;
import frc.robot.interfaces.LogDestination;

/**
 * General utilities used by all classes that implementate IDataLogger 
 */
public class LoggingUtilities {

    // the intention is run this at robot startup so probing the paths cannot affect loops
	public static String CheckLogDestination(LogDestination logDestination) {
		String logFolderPath = "";

		switch(logDestination)
		{
			case NONE:
				System.out.println("-------------------------------------");
				System.out.println("..Logging Disabled");
				System.out.println("-------------------------------------");
				break;

			case USB:
				// see if the USB stick is plugged into to RoboRIO
				Path path = Paths.get(RobotMap.PRIMARY_LOG_FILE_PATH);
				Path alternatePath = Paths.get(RobotMap.ALTERNATE_LOG_FILE_PATH);

				if (Files.exists(path)) {
					logFolderPath = path.toString();
				}
				else if (Files.exists(alternatePath)) {
					logFolderPath = alternatePath.toString();
				}
				else {
					System.out.println("-------------------------------------");
					System.out.println("..Error configuring Logging to USB");
					System.out.println("-------------------------------------");	
				}

				break;

			case ROBORIO:
				break;
		}

		if (!GeneralUtilities.isStringEmpty(logFolderPath))
		{
			System.out.println("-------------------------------------");
			System.out.println("..Log file will be created in folder: " + logFolderPath);
			System.out.println("-------------------------------------");			
		}

		return logFolderPath;
	}

	public static PrintWriter CreateLogWriter(String logFolderPath, String logFileSuffix) {

		SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
		outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
		String newDateString = outputFormatter.format(new Date());
    	
    	// build the new filename
		String fileName = newDateString + "_" + logFileSuffix + ".tsv";
		
    	// build the full file path name
		String logFilePathName = logFolderPath + File.separator + fileName;
		
		PrintWriter writer = null;
		try {
			writer = new PrintWriter(new BufferedWriter(new FileWriter(logFilePathName, true)));

			System.out.println("-------------------------------------");
			System.out.println("..Logging enabled to: " + logFilePathName);
			System.out.println("-------------------------------------");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();

			System.out.println("-------------------------------------");
			System.out.println("..Error configuring Logging to: " + logFilePathName);
			System.out.println("-------------------------------------");
		}

		return writer;
	}

}
