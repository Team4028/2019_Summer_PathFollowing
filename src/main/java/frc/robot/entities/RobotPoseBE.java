/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.entities;

import frc.robot.Robot;
import frc.robot.subsystems.Chassis;

/**
 * Add your docs here.
 */
public class RobotPoseBE {
    public final double LeftPositionInInches;
    public final double LeftXInInches;
    public final double LeftYInInches;
    
    public final double RightPositionInInches;
    public final double RightXInInches;
    public final double RightYInInches;

    public final double HeadingInDegrees;
    

    public RobotPoseBE() {
        LeftPositionInInches = 0;
        LeftXInInches = 0;
        LeftYInInches = 0;
        
        RightPositionInInches = 0;
        RightXInInches = 0;
        RightYInInches = 0;
    
        HeadingInDegrees = 0;
    }
    

    public RobotPoseBE(double leftPositionInInches, double leftXInInches, double leftYInInches, 
                        double rightPositionInInches, double rightXInInches, double rightYInInches, 
                        double headingInDegrees) {
        LeftPositionInInches = leftPositionInInches;
        LeftXInInches = leftXInInches;
        LeftYInInches = leftYInInches;
        
        RightPositionInInches = rightPositionInInches;
        RightXInInches = rightXInInches;
        RightYInInches = rightYInInches;
    
        HeadingInDegrees = headingInDegrees;
    }

    public static RobotPoseBE init()
    {
        return new RobotPoseBE(0.0, 
                                0.0, 
                                0 - Chassis.TRACK_WIDTH_INCHES / 2.0,
                                0.0, 
                                0.0, 
                                0 + Chassis.TRACK_WIDTH_INCHES / 2.0, 
                                0.0);
    }
}
