/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.entities.RobotPoseBE;

/**
 * This class estimates the Robots new Pose based on its incremental movement from a previous reference position
 * 
 * https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf
 * http://team449.shoutwiki.com/wiki/Pose_Estimation
 * 
 * EXPLANATION OF STATE ESTIMATION
 * Since we don't have the (x,y) coordinates of each side of the chassis, we need to calculate them in order
 * to be able to map graphically the trajectory of the chassis. The information we do have is position and 
 * heading at every scan cycle, so using Euler's Method (Calculus stuff), we can add successive vectors to
 * closely map the actual trajectory (nearly 100% resolution)
 */
public class PoseEstimationV1 {

    public static RobotPoseBE EstimateNewPose(RobotPoseBE previousPose, double currentLeftPositionInInches, double currentRightPositionInInches, 
                                                double currentHeadingInDegrees) {

        // calc deltas from last position
        double leftDeltaPositionInInches = (currentLeftPositionInInches - previousPose.LeftPositionInInches);
        double rgtDeltaPositionInInches = (currentRightPositionInInches - previousPose.RightPositionInInches);
        double deltaHeadingInDegrees = (currentHeadingInDegrees - previousPose.HeadingInDegrees);

        double avgNewHeadingInDegree = Math.toRadians(previousPose.HeadingInDegrees) + (deltaHeadingInDegrees / 2.0);

        // calc left x&y position, remember args to cos() & sin() is in radians!
        double leftDeltaXInInches = leftDeltaPositionInInches * Math.cos(avgNewHeadingInDegree);
        double leftDeltaYInInches = leftDeltaPositionInInches * Math.sin(avgNewHeadingInDegree);

        // calc right x&y position, remember args to cos() & sin() is in radians!
        double rgtDeltaXInInches = rgtDeltaPositionInInches * Math.cos(avgNewHeadingInDegree);
        double rgtDeltaYInInches = rgtDeltaPositionInInches * Math.sin(avgNewHeadingInDegree);

        return new RobotPoseBE(currentLeftPositionInInches,
                                previousPose.LeftXInInches + leftDeltaXInInches,
                                previousPose.LeftYInInches + leftDeltaYInInches,
                                currentRightPositionInInches,
                                previousPose.RightXInInches + rgtDeltaXInInches,
                                previousPose.RightYInInches + rgtDeltaYInInches,
                                currentHeadingInDegrees);
    }
}
