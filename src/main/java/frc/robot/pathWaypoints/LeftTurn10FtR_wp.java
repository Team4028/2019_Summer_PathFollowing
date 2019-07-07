/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.pathWaypoints;

import frc.robot.subsystems.Chassis;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class LeftTurn10FtR_wp {

    /*
    X,      Y,      Tangent X,          Tangent Y,  Fixed Theta,    Name
    0.0,    300.0,  120.0,              0.0,        true,
    120.0,  180.0,  0.0,                -120.0,     true,
    */

    private static final Waypoint[] points = new Waypoint[] {
        new Waypoint(0, 300.0, Pathfinder.d2r(0)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
        new Waypoint(120.0, 180.0, Pathfinder.d2r(-90.0)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
    };

    private Trajectory _leftTrajectory = null;
    private Trajectory _rightTrajectory = null;

    private static final String PATH_NAME = "LeftTurn10ftR";

    public LeftTurn10FtR_wp()
    {
        // Create the Trajectory Configuration
        //
        // Arguments:
        // Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
        // Sample Count:        SAMPLES_HIGH (100 000)
        //                      SAMPLES_LOW  (10 000)
        //                      SAMPLES_FAST (1 000)
        // Time Step:           0.01 Seconds
        // Max Velocity:        125 in/sec
        // Max Acceleration:    150 in/sec/sec
        // Max Jerk:            60.0 in/s/s/s
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, 125.0, 150.0, 60.0);

        // Generate the trajectory
        Trajectory trajectory = Pathfinder.generate(points, config);

        // The distance between the left and right sides of the wheelbase is 0.6m
        double wheelbase_width = Chassis.TRACK_WIDTH_INCHES;

        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);

        // Generate the Left and Right trajectories using the original trajectory
        // as the centre
        modifier.modify(wheelbase_width);

        _leftTrajectory  = modifier.getLeftTrajectory();       // Get the Left Side
        _rightTrajectory = modifier.getRightTrajectory();      // Get the Right Side

        System.out.println("-------------------------------------");
        System.out.println("... Generated Path: " + PATH_NAME);
        System.out.println("-------------------------------------");
    }

    public Trajectory get_leftTrajectory() {
        return _leftTrajectory;
    }

    public Trajectory get_rightTrajectory() {
        return _rightTrajectory;
    }
}
