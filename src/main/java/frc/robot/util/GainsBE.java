/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * This class holds a set of PID Gain constants
 */
public class GainsBE 
{
	public final double KP;
	public final double KI;
	public final double KD;
	public final double KF;
	public final int KMaxI;
	public final double KPeakOutput;

    public GainsBE(double _kP, double _kI, double _kD, double _kF, int _kMaxI, double _kPeakOutput)
    {
		KP = _kP;
		KI = _kI;
		KD = _kD;
		KF = _kF;
		KMaxI = _kMaxI;
		KPeakOutput = _kPeakOutput;
	}
}
