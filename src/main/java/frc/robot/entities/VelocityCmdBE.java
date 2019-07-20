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

    public final double MtrPPosCmd;
    public final double MtrIPosCmd;
    public final double MtrPVelCmd;
    public final double MtrFFVelCmd;
    public final double MtrFFAccelCmd;

    //private double _mtrTurnAdjCmd;
    private double _otherChassisSideRawFinalMtrCmd; // to support saturation scal

    public final double PositionErrorInInches;
    public final double VelocityErrorInIPS;
    public final Segment CurrentSegment;
    private final double MIN_MTR_CMD_DEADBAND = 0.015;

    public VelocityCmdBE() {
        this.MtrPPosCmd = 0;
        this.MtrIPosCmd = 0;
        this.MtrPVelCmd = 0;
        this.MtrFFVelCmd = 0;
        this.MtrFFAccelCmd = 0;  

        this.PositionErrorInInches = 0;
        this.VelocityErrorInIPS = 0;
        this.CurrentSegment = null;
    }

    public VelocityCmdBE(double mtrPPosCmd, double mtrPVelCmd, double mtrIPosCmd, double mtrFFVelCmd, double mtrFFAccelCmd, double positionErrorInInches, double velocityErrorInIPS, Segment currentSegment) {
        this.MtrPPosCmd = mtrPPosCmd;
        this.MtrPVelCmd = mtrPVelCmd;
        this.MtrIPosCmd = mtrIPosCmd;
        this.MtrFFVelCmd = mtrFFVelCmd;
        this.MtrFFAccelCmd = mtrFFAccelCmd;

        this.PositionErrorInInches = positionErrorInInches;
        this.VelocityErrorInIPS = velocityErrorInIPS;
        this.CurrentSegment = currentSegment;
    }

    //public void set_mtrTurnAdjCmd(double mtrTurnAdjCmd) {
    //    _mtrTurnAdjCmd = mtrTurnAdjCmd;
    //}

    //public double get_mtrTurnAdjCmd() {
    //    return _mtrTurnAdjCmd;
    //}

    public double get_RawBaseMtrCmd() {
        double baseMtrCmd = MtrPPosCmd + MtrIPosCmd + MtrPVelCmd + MtrFFVelCmd + MtrFFAccelCmd;

        return baseMtrCmd;
    }

    public double get_AdjBaseMtrCmd() {
        double adjBaseMtrCmd = get_RawBaseMtrCmd();

        if(adjBaseMtrCmd > .01 && adjBaseMtrCmd< .18)
        {
            adjBaseMtrCmd = .18;
        }
        // TODO: decide if we need this
        return adjBaseMtrCmd;
    }

    public double get_RawFinalMtrCmd() {
        return (get_AdjBaseMtrCmd()); // + get_mtrTurnAdjCmd());
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
