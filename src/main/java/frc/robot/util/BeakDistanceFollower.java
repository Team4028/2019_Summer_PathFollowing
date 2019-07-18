/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.entities.VelocityCmdBE;
import jaci.pathfinder.Trajectory;

/**
 * This is based on Jaci's class with a few tweaks
 */
public class BeakDistanceFollower {

    double _kP, _kI, _kD, _kV, _kA;

    double _oP, _oI, _oD, _oV, _oA;

    double _lastPositionError, _targetHeadingInRadians, _lastSegmentAccel, _calculatedMtrCmd;

    private final double MIN_ACCEL_CHG = 30.0;

    int _segmentIdx;
    Trajectory _trajectory;

    // constructors
    public BeakDistanceFollower(Trajectory traj) {
        this._trajectory = traj;
    }

    public BeakDistanceFollower() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     * @param traj a previously generated trajectory
     */
    public void setTrajectory(Trajectory traj) {
        this._trajectory = traj;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
        this._kP = kp; 
        this._kI = ki; 
        this._kD = kd;
        this._kV = kv; 
        this._kA = ka;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        _lastPositionError = 0; 
        _segmentIdx = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the distance the robot has covered.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param distance_covered  The distance covered in meters
     * @return                  The desired output for your motor controller
     */
    public VelocityCmdBE calculate(double distance_covered) {
        if (_segmentIdx < _trajectory.length()) {
            Trajectory.Segment currentSegment = _trajectory.get(_segmentIdx);
            double currentPositionError = currentSegment.position - distance_covered;
            double chgPositionError = currentPositionError - _lastPositionError;

            /*
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / seg.dt) +          // Derivative
                    (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
            */

            // calc components of velocity command
            //_oP = (currentPositionError > 0) ?
            //        _kP * currentPositionError
            //        : 0;
            _oP = _kP * currentPositionError;
            _oI = 0;
            _oD = (chgPositionError > 0) ?
                    _kD * (chgPositionError / currentSegment.dt)
                    : 0;
            _oV = _kV * currentSegment.velocity;
            _oA = ((currentSegment.acceleration - _lastSegmentAccel) > MIN_ACCEL_CHG) ?
                     _kA * currentSegment.acceleration 
                     : 0;
            //_oA = _kA * currentSegment.acceleration;

            /*
            double calculated_value =
                    _kP * currentPositionError +                                    // Proportional
                    _kD * ((currentPositionError - _lastPositionError) / seg.dt) +          // Derivative
                    (_kV * seg.velocity + _kA * seg.acceleration);    // V and A Terms
            */
            _calculatedMtrCmd = (_oP + _oD + _oV + _oA);

            _lastPositionError = currentPositionError;
            _lastSegmentAccel = currentSegment.acceleration;
            _targetHeadingInRadians = currentSegment.heading;

            _segmentIdx++;

            return new VelocityCmdBE(_oP, _oI, _oD, _oV, _oA, currentPositionError, currentSegment);
        } 
        else {
            return new VelocityCmdBE();
        }
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getTargetHeadingInRadians() {
        return _targetHeadingInRadians;
    }

    public double getLastPositionError() {
        return _lastPositionError;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return _trajectory.get(_segmentIdx);
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return _segmentIdx >= _trajectory.length();
    }

}
