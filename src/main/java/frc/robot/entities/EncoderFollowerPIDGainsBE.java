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
public class EncoderFollowerPIDGainsBE {
	public final double kPPosErr;
	public final double kPVelErr;
	public final double kIPosErr;
	public final double kDPosErr;
	public final double kFFVelCmd;
	public final double kFFAccelCmd;

	// constructor(s)
	// @param _kPPosErr		The proportional gain on position error.
	// @param _kPVelErr		The integral gain on velocity error.
	// @param _kIPosErr		The integral gain on position error.
	// @param _kDPosErr		The derivative gain on position error.
	// @param _kFFVelCmd	The feedforward gain on velocity.
	// @param _kFFAccelCmd	The feedforward gain on acceleration.
    public EncoderFollowerPIDGainsBE(double _kPPosErr, double _kPVelErr, double _kIPosErr, double _kDPosErr, double _kFFVelCmd, double _kFFAccelCmd) {
		kPPosErr = _kPPosErr;
		kPVelErr = _kPVelErr;
		kIPosErr = 0;
		kDPosErr = _kDPosErr;
		kFFVelCmd = _kFFVelCmd;
		kFFAccelCmd = _kFFAccelCmd;
	}
}
