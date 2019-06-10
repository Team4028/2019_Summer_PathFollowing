/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class EncoderFollowerPIDGainsBE 
{
    public final double KP;
	public final double KI;
	public final double KD;
	public final double KV;
	public final double KA;

    public EncoderFollowerPIDGainsBE(double _kP, double _kI, double _kD, double _kV, int _kA)
    {
		KP = _kP;
		KI = _kI;
		KD = _kD;
		KV = _kV;
		KA = _kA;
	}
}
