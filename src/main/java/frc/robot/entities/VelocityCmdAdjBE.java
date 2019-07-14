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
    public final double HeadingErrorInDegrees;

    //public double LeftPosErrInInches;
    //public double RgtPosErrInInches;

    public final String TurnDirection;
    public final String TurnCmdChg;


    public final double LeftMtrCmdTurnAdj;
    public final double RgtMtrCmdTurnAdj;

    public VelocityCmdAdjBE(double leftMtrCmdTurnAdj, double rgtMtrCmdTurnAdj, double headingErrorInDegrees, String turnDirection, String turnCmdChg) {
        LeftMtrCmdTurnAdj = leftMtrCmdTurnAdj;
        RgtMtrCmdTurnAdj = rgtMtrCmdTurnAdj;
        HeadingErrorInDegrees = headingErrorInDegrees;
        TurnDirection = turnDirection;
        TurnCmdChg = turnCmdChg;
    }

    public static VelocityCmdAdjBE init() {
        return new VelocityCmdAdjBE(0.0, 0.0, 0.0, "", "");
    }

}
