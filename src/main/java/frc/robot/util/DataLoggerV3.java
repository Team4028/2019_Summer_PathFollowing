/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.entities.LogDataBE;
import frc.robot.interfaces.IDataLogger;

/**
 * This version tries to make logging totally async to the control loops
 * 	(gathering the event data and writing is async)
 * 
 */
public class DataLoggerV3 implements IDataLogger {

    @Override
    public void initLogging(String robotMode) {

    }

    @Override
    public void LogData(LogDataBE logData) {

    }

    @Override
    public void close() {

    }

    @Override
    public boolean get_isLoggingEnabled() {
        return false;
    }

    @Override
    public void setMarker(String markerName) {

    }

    @Override
    public void clearMarker() {

    }
}
