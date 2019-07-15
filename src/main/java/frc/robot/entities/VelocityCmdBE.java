/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.entities;

import jaci.pathfinder.Trajectory.Segment;

/**
 * Add your docs here.
 */
public class VelocityCmdBE {

    public final double MtrPCmd;
    public final double MtrICmd;
    public final double MtrDCmd;
    public final double MtrVCmd;
    public final double MtrACmd;

    private double _mtrTurnAdjCmd;
    private double _mtrStictionAdjCmd;  // thisis equiv to VIntercept
    private double _otherChassisSideRawFinalMtrCmd; // to support saturation scal

    public final double PositionErrorInInches;
    public final Segment CurrentSegment;
    private final double MIN_MTR_CMD_DEADBAND = 0.015;

    public VelocityCmdBE() {
        this.MtrPCmd = 0;
        this.MtrICmd = 0;
        this.MtrDCmd = 0;
        this.MtrVCmd = 0;
        this.MtrACmd = 0;  

        this.PositionErrorInInches = 0;
        this.CurrentSegment = null;
    }

    public VelocityCmdBE(double mtrPCmd, double mtrICmd, double mtrDCmd, double mtrVCmd, double mtrACmd, double positionErrorInInches, Segment currentSegment) {
        this.MtrPCmd = mtrPCmd;
        this.MtrICmd = mtrICmd;
        this.MtrDCmd = mtrDCmd;
        this.MtrVCmd = mtrVCmd;
        this.MtrACmd = mtrACmd;

        this.PositionErrorInInches = positionErrorInInches;
        this.CurrentSegment = currentSegment;
    }

    public void set_mtrStictionAdjCmd(double mtrStictionAdjCmd) {
        _mtrStictionAdjCmd = mtrStictionAdjCmd;
    }

    public void set_mtrTurnAdjCmd(double mtrTurnAdjCmd) {
        _mtrTurnAdjCmd = mtrTurnAdjCmd;
    }

    public double get_mtrTurnAdjCmd() {
        return _mtrTurnAdjCmd;
    }

    public double get_RawBaseMtrCmd() {
        double baseMtrCmd = MtrPCmd + MtrICmd + MtrDCmd + MtrVCmd + MtrACmd;

        return baseMtrCmd;
    }

    public double get_AdjBaseMtrCmd() {
        double adjBaseMtrCmd = get_RawBaseMtrCmd();

        // if mtr cmd > min threshhold && <= kIntercept then force to kIntercept value
        if ((adjBaseMtrCmd > MIN_MTR_CMD_DEADBAND) && (adjBaseMtrCmd <= _mtrStictionAdjCmd))
        {
            adjBaseMtrCmd = _mtrStictionAdjCmd;
        }
        else if ((adjBaseMtrCmd > (-1.0 * MIN_MTR_CMD_DEADBAND)) && (adjBaseMtrCmd <= (-1.0 * _mtrStictionAdjCmd)))
        {
            adjBaseMtrCmd = -1.0 * _mtrStictionAdjCmd;
        }

        return adjBaseMtrCmd;
    }

    public double get_RawFinalMtrCmd() {
        return (get_AdjBaseMtrCmd() + get_mtrTurnAdjCmd());
    }

    public double get_ScaledFinalMtrCmd() {
        return (get_RawFinalMtrCmd() / get_MtrScaleFactor());
    }

    //actual calc is defered to get_MtrScaleFactor() method
    public void calcMtrScaleFactor(double otherChassisSideRawFinalMtrCmd) {
        _otherChassisSideRawFinalMtrCmd = otherChassisSideRawFinalMtrCmd;
    }

    public double get_MtrScaleFactor() {
        double mtrScaleFactor = 1.0;

        // we only need to scale if one of the sides is saturated
        if(Math.abs(_otherChassisSideRawFinalMtrCmd) > 1.0 || Math.abs(this.get_RawFinalMtrCmd()) > 1.0) {
            mtrScaleFactor = (Math.abs(_otherChassisSideRawFinalMtrCmd) > Math.abs(this.get_RawFinalMtrCmd()))
                                ? Math.abs(_otherChassisSideRawFinalMtrCmd)
                                : Math.abs(this.get_RawFinalMtrCmd());
            
        }

        return (mtrScaleFactor);
    }
}
