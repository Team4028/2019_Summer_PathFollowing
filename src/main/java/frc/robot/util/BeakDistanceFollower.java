/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.entities.EncoderFollowerPIDGainsBE;
import frc.robot.entities.VelocityCmdBE;
import jaci.pathfinder.Trajectory;

/**
 * This is based on Jaci's class with a few tweaks
 */
public class BeakDistanceFollower {

    double _kPPosErr, _kPVelErr, _kIPosErr, _kFFVelCmd, _kFFAccelCmd, _kInterceptCmd;

    //double _oPPosErrCmd, _oPVelErrCmd, _oIPosErrCmd, _oFFVelCmd, _oFFAccelCmd, _oInterceptCmd;

    double _lastPositionError, _targetHeadingInRadians, _lastSegmentAccel;

    int _segmentIdx;
    Trajectory _trajectory;

    private static final double MIN_MOTOR_CMD = 0.01;
    private static final double MIN_DECEL_VEL_IN_IPS = 5.0;    // point that KI & KPV start oscillating

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
     * @param kp    The proportional gain on position error.
     * @param ki    The integral gain on position error.
     * @param kv    The proportional gain on velocity error (or derivative gain on position error).
     * @param kffv  The feedforward gain on velocity. Should be 1.0 if the units of the profile match the units of the output.
     * @param kffa  The feedforward gain on acceleration.
     */
    public void configurePIDVA(double kPPosErr, double kPVelErr, double kIPosErr, double kFFVelCmd, double kFFAccelCmd, double kInterceptCmd) {
        this._kPPosErr = kPPosErr; 
        this._kPVelErr = kPVelErr; 
        this._kIPosErr = kIPosErr;
        this._kFFVelCmd = kFFVelCmd; 
        this._kFFAccelCmd = kFFAccelCmd;
        this._kInterceptCmd = kInterceptCmd;
    }

    public void configurePIDVA(EncoderFollowerPIDGainsBE gains, double kInterceptCmd) {
        this._kPPosErr = gains.kPPosErr; 
        this._kPVelErr = gains.kPVelErr; 
        this._kIPosErr = gains.kIPosErr;
        this._kFFVelCmd = gains.kFFVelCmd; 
        this._kFFAccelCmd = gains.kFFAccelCmd;
        this._kInterceptCmd = kInterceptCmd;
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
     * @param distanceCovered  The distance covered in meters
     * @return                  The desired output for your motor controller
     */
    public VelocityCmdBE calculate(double distanceCovered, double currentVelocity) {
        if (_segmentIdx < _trajectory.length()) {
            // get current segment
            Trajectory.Segment currentSegment = _trajectory.get(_segmentIdx);

            // calc erros
            double currentPositionError = currentSegment.position - distanceCovered;
            double currentVelocityError = currentSegment.velocity - currentVelocity;

            // calc components of velocity command
            double oPPosErrCmd = _kPPosErr * currentPositionError;
            double oPVelErrCmd = (((currentSegment.acceleration < 0) 
                                    && (currentVelocityError < 0.0))    // we are decel + too fast
                                   || ((currentSegment.acceleration > 0) 
                                    && (currentVelocityError > 0.0)))   // we are accel + too slow
                                    ? _kPVelErr * currentVelocityError 
                                    : 0;
            double oIPosErrCmd = 0;

            double oFFVelCmd = _kFFVelCmd * currentSegment.velocity;
            double oFFAccelCmd = _kFFAccelCmd * currentSegment.acceleration;

            double rawCmd = (oPPosErrCmd
                                + oPVelErrCmd
                                + oIPosErrCmd
                                + oFFVelCmd
                                + oFFAccelCmd);

            double oInterceptCmd = 0;
            if (((currentSegment.acceleration >= 0) && (rawCmd > MIN_MOTOR_CMD))
                || ((currentSegment.acceleration < 0) && (currentSegment.velocity > MIN_DECEL_VEL_IN_IPS))) {
                oInterceptCmd = _kInterceptCmd;
            }

            _targetHeadingInRadians = currentSegment.heading;
            _segmentIdx++;

            return new VelocityCmdBE(oPPosErrCmd, oPVelErrCmd, oIPosErrCmd, oFFVelCmd, oFFAccelCmd, oInterceptCmd, currentPositionError, currentVelocityError, currentSegment);
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
