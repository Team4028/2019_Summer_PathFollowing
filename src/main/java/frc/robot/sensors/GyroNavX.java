/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.entities.LogDataBE;

/**
 * This class exposes the OnBoard Navigation Sensor 
 * 
 * Lead Student:
 * 
 * Press the ‘CAL’ button on the navX-MXP Circuit board AND HOLD THE BUTTON DOWN FOR AT LEAST 5 SECONDS.
 * Release the ‘CAL’ button, and verify that the orange ‘CAL’ light flashes for 1 second and then turns off.
 * Press the ‘RESET’ button on the navX-MXP circuit board, causing it to restart.
    The navX-MXP circuit board will now begin OmniMount auto-calibration. During this auto-calibration period, the orange ‘CAL’ LED will flash repeatedly.  This process takes approximately 15 seconds, and requires two things: 
    1.  During auto-calibration, one of the navX-MXP axes MUST be perpendicular to the earth’s surface.
    2.  During auto-calibration, navX-MXP must be held still.
 * If either of the above conditions is not true, the ‘CAL’ LED will be flashing quickly, indicating an error.  To resolve this error, you must ensure that conditions 1 and 2 are met, at which point the ‘CAL’ LED will begin flashing slowly, indicating calibration is underway.
 * Once navX-MXP auto-calibration is complete, the Board Frame to Body Frame Transform will be stored persistently into navX-MXP flash memory and used until auto-calibration is run once again.
 */
public class GyroNavX implements IBeakSquadDataPublisher {
    // define class level working variables
    private AHRS _navXSensor;

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static GyroNavX _instance = new GyroNavX();

    public static GyroNavX getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private GyroNavX() {
        try {
            _navXSensor = new AHRS(RobotMap.NAVX_PORT); // Communication via RoboRIO MXP (SPI)
            DriverStation.reportWarning("NavX connected.", false);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
        }
    }

    public double getRawYaw() {
        return _navXSensor.getYaw();
    }

    public void zeroYaw() {
        _navXSensor.zeroYaw();
    }

    // If you have a typical gyro, then it gives a + reading for a clockwise rotation \
    public double getHeadingInDegrees() {
        // we use YAW baswd on the mounting orientation of the NavX
        return _navXSensor.getYaw();
    }

    public double getHeadingInRadians() {
        return Math.toRadians(getHeadingInDegrees());
    }

    public double getPitch() {
        // Axis Perpendicular to the Front/Back of the robot
        return _navXSensor.getPitch();
    }

    // =====================================================================================
    // Helper Methods
    // =====================================================================================
    public void updateDashboard() {
        SmartDashboard.putNumber("GyroNavX:RawYaw", getRawYaw());
    }

    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggingEnabled) 
    {
        logData.AddData("GyroNavX:RawYaw", Double.toString(getRawYaw()));
    }
}
