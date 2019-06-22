/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.entities;

/**
 * Class to hold gains for EncoderFollower
 * KI commented out since I Gain is not supported by Jaci's PathFollower
 */
public class EncoderFollowerPIDGainsBE 
{
    public final double KP;
	public final double KI;
	public final double KD;
	public final double KV;
	public final double KA;

	// constructor(s)
    public EncoderFollowerPIDGainsBE(double _kP, double _kD, double _kV, double _kA)
    {
		KP = _kP;
		KI = 0;
		KD = _kD;
		KV = _kV;
		KA = _kA;
	}
}
