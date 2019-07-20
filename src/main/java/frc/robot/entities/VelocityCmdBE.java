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

    public final double MtrKPPosCmd;
    public final double MtrKIPosCmd;
    public final double MtrKPVelCmd;
    public final double MtrKFFVelCmd;
    public final double MtrKFFAccelCmd;
    public final double MtrKIntereptCmd;

    //private double _mtrTurnAdjCmd;
    private double _otherChassisSideRawFinalMtrCmd; // to support saturation scal

    public final double PositionErrorInInches;
    public final double VelocityErrorInIPS;
    public final Segment CurrentSegment;



    public VelocityCmdBE() {
        this.MtrKPPosCmd = 0;
        this.MtrKIPosCmd = 0;
        this.MtrKPVelCmd = 0;
        this.MtrKFFVelCmd = 0;
        this.MtrKFFAccelCmd = 0;  
        this.MtrKIntereptCmd = 0;

        this.PositionErrorInInches = 0;
        this.VelocityErrorInIPS = 0;
        this.CurrentSegment = null;
    }

    public VelocityCmdBE(double mtrkPPosCmd, double mtrkPVelCmd, double mtrkIPosCmd, double mtrkFFVelCmd, double mtrkFFAccelCmd, double mtrKIntereptCmd, double positionErrorInInches, double velocityErrorInIPS, Segment currentSegment) {
        this.MtrKPPosCmd = mtrkPPosCmd;
        this.MtrKPVelCmd = mtrkPVelCmd;
        this.MtrKIPosCmd = mtrkIPosCmd;
        this.MtrKFFVelCmd = mtrkFFVelCmd;
        this.MtrKFFAccelCmd = mtrkFFAccelCmd;
        this.MtrKIntereptCmd = mtrKIntereptCmd;

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
        double baseMtrCmd = MtrKPPosCmd + MtrKIPosCmd + MtrKPVelCmd + MtrKFFVelCmd + MtrKFFAccelCmd;

        return baseMtrCmd;
    }

    public double get_AdjBaseMtrCmd() {
        double adjBaseMtrCmd = get_RawBaseMtrCmd() + MtrKIntereptCmd;

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
