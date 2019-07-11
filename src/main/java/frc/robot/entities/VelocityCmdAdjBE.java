/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.entities;

/**
 * Add your docs here.
 */
public class VelocityCmdAdjBE {
    public final double LeftMtrCmdTurnAdj;
    public final double RgtMtrCmdTurnAdj;
    public final double HeadingErrorInDegrees;

    public double LeftBaseMtrCmd;
    public double RgtBaseMtrCmd;


    public VelocityCmdAdjBE(double leftMtrCmdTurnAdj, double rgtMtrCmdTurnAdj, double headingErrorInDegrees) {
        LeftMtrCmdTurnAdj = leftMtrCmdTurnAdj;
        RgtMtrCmdTurnAdj = rgtMtrCmdTurnAdj;
        HeadingErrorInDegrees = headingErrorInDegrees;
    }

    public static VelocityCmdAdjBE init() {
        return new VelocityCmdAdjBE(0.0, 0.0, 0.0);
    }

    public double LeftFinalMtrCmd() {
        return LeftBaseMtrCmd + LeftMtrCmdTurnAdj;
    }

    public double RgtFinalMtrCmd() {
        return RgtBaseMtrCmd + RgtMtrCmdTurnAdj;
    }
}
