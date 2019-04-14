/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.interfaces;

import frc.robot.util.LogDataBE;

/**
 * This interface defines the methods all Subsystems classes must implement
 */
public interface IBeakSquadDataPublisher {
    public void updateLogData(LogDataBE logData);
	
	public void updateDashboard();
}
