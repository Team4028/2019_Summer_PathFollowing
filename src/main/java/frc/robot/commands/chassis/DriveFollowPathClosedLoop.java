/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import static jaci.pathfinder.Pathfinder.r2d;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.entities.EncoderFollowerPIDGainsBE;
import frc.robot.entities.LogDataBE;
import frc.robot.util.GeneralUtilities;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path using Velocity Closed Loop on the TalonSRX and Notifier for the RoboRio loop
public class DriveFollowPathClosedLoop extends Command implements IBeakSquadDataPublisher {

    // working variables
    private Chassis _chassis = Robot._Chassis;
    private GyroNavX _navX = Robot._navX;

    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    // create notifier that will 
    private Notifier _notifier = new Notifier(this::followPath);

    // The starting positions of the left and right sides of the drivetrain
    private double _leftStartingDistance = 0;
    private double _rightStartingDistance = 0;

    private Runnable _loggingMethodDelegate;
    private double _loopPeriodInMS = 0;
    private double _lastLoopTimeInMS = 0;

    private StringBuilder _sb = new StringBuilder();

    /*
     * Equation: output = (kP * error) + (kD * diff(error)/dt) + (kV * vel) + (kA * accel)
     * 
     * kP -> Proportional gain. Units: %/m. I typically start off with this at
     * around 0.8-1.2, using a standard AM drivebase.
     * 
     * kI -> pathfinder ignores kI
     * 
     * kD -> Derivative gain. Units: %/(m/s). Iâ€™ll typically only tune this if
     * tracking is bad, so usually Iâ€™ll keep it at 0 unless I have a reason to
     * change it. You can think of it like a way to increase the value that the kV
     * term puts out, which can help in the lower velocity ranges if your
     * acceleration is bad.
     * 
     * kV -> Velocity Feed-forward gain. Units: %/(m/s). This should be
     * max_%/max_vel (i.e. 1 / max_vel). This is used to give the loop some kind of
     * knowledge about what its velocity should be. As Oblarg mentioned, in other
     * implementations (i.e. Talon SRX) this will be calculated differently.
     * 
     * kA -> Acceleration Feed-forward gain. Units: %/(m/s/s). I typically leave
     * this value at 0, but you can adjust it if youâ€™re unhappy with the speed of
     * your robot and need more power in the acceleration phase(s) of the loop. This
     * value can also be pretty dangerous if you tune it too high.
     */
    private final static EncoderFollowerPIDGainsBE _leftFollowerGains 
                            = new EncoderFollowerPIDGainsBE(0.0, 0.0, 1.0, 0.0);

    private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
                            = new EncoderFollowerPIDGainsBE(0.0, 0.0, 1.0, 0.0);

    // ======================================================================================
    // constructor
    // ======================================================================================
    public DriveFollowPathClosedLoop(String pathName, Runnable loggingMethodDelegate) {
        // Use requires() here to declare subsystem dependencies
        requires(_chassis);
        setInterruptible(true);

        importPath(pathName);

        _leftFollower.configurePIDVA(_leftFollowerGains.KP, 
                                        _leftFollowerGains.KI, 
                                        _leftFollowerGains.KD,
                                        _leftFollowerGains.KV, 
                                        _leftFollowerGains.KA);

        _rightFollower.configurePIDVA(_rightFollowerGains.KP, 
                                        _rightFollowerGains.KI, 
                                        _rightFollowerGains.KD,
                                        _rightFollowerGains.KV, 
                                        _rightFollowerGains.KA);

        // save a local reference to the delegate to the logging method
        _loggingMethodDelegate = loggingMethodDelegate;
    }

    @Override
    protected void initialize() {
        // Set the starting positions of the left and right sides of the drivetrain
        _leftStartingDistance = _chassis.getLeftChassisPositionInInches();
        _rightStartingDistance = _chassis.getRightChassisPositionInInches();

        // Make sure we're starting at the beginning of the path
        _leftFollower.reset();
        _rightFollower.reset();

        // set chassis pid constants
        _chassis.setActivePIDConstantsSlot(1);

        // initialize last loop timestamp
        _lastLoopTimeInMS = RobotController.getFPGATime() / 1000.0;

        // Start running the path
        _notifier.startPeriodic(_loopPeriodInMS);
    }

    @Override
    protected boolean isFinished() {
        return _leftFollower.isFinished() || _rightFollower.isFinished();
    }

    // Commmand ended normally (typically because isFinished returns true)
    @Override
    protected void end() {
        _notifier.stop();
        _chassis.stop(false);
    }

    //
    @Override
    protected void interrupted() {
        _notifier.stop();
        _chassis.stop(false);
    }

    // name of this path
    // https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java
    // https://wpilib.screenstepslive.com/s/currentCS/m/84338/l/1021631-integrating-path-following-into-a-robot-program
    // https://www.chiefdelphi.com/t/tuning-pathfinder-pid-talon-motion-profiling-magic-etc/162516
    private void importPath(String pathName) {
        try {
            // Read the path files from the file system
            // Note: Bug in this version of PathFollower, generated paths are in wrong filename
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".right");
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".left");

            // Set the two paths in the followers
            _leftFollower = new DistanceFollower(leftTrajectory);
            _rightFollower = new DistanceFollower(rightTrajectory);

            _loopPeriodInMS = leftTrajectory.get(0).dt;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void followPath() {

        // if a logging method delegate was passed in, call it to log the current values
        //  likely the result of the provious loop
        if (_loggingMethodDelegate != null) {
            _loggingMethodDelegate.run();
        }

        double currentTimeInMS = RobotController.getFPGATime() / 1000.0;
        System.out.println(GeneralUtilities.roundDouble(currentTimeInMS - _lastLoopTimeInMS, 1));
        _lastLoopTimeInMS = currentTimeInMS;

        // Get the left and right power output from the distance calculator
        double left_speed = 
                _leftFollower.calculate(_chassis.getLeftChassisPositionInInches() - _leftStartingDistance);
        double right_speed = 
                _rightFollower.calculate(_chassis.getRightChassisPositionInInches() - _rightStartingDistance);

        // Calculate any correction we need based on the current and desired heading
        double heading = _navX.getPathfinderYaw();
        double desired_heading = r2d(_leftFollower.getHeading());
        double heading_difference = 0.0; // boundHalfDegrees(desired_heading - heading);
        double turn = 0.8 * (-1.0 / 80.0) * heading_difference;

        // Send the % outputs to the drivetrain
        _chassis.setClosedLoopVelocityCmd(left_speed + turn, right_speed - turn);
    }

    @Override
    public void updateLogData(LogDataBE logData) {
        if (!_leftFollower.isFinished()) 
        {
            _sb.setLength(0);
            _sb.append(Double.toString(_leftFollowerGains.KP));
            _sb.append(" | ");
            _sb.append(Double.toString(_leftFollowerGains.KI)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_leftFollowerGains.KD)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_leftFollowerGains.KV)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_leftFollowerGains.KA));

            logData.AddData("LeftFollower:Gains", _sb.toString());
    
            Segment currentLeftSegment = _leftFollower.getSegment();
            logData.AddData("LeftFollower:SegmentPos", Double.toString(currentLeftSegment.position));
            logData.AddData("LeftFollower:SegmentVel", Double.toString(currentLeftSegment.velocity));
            logData.AddData("LeftFollower:SegmentAccel", Double.toString(currentLeftSegment.acceleration));
        }
    
        if (!_rightFollower.isFinished()) 
        {
            _sb.setLength(0);
            _sb.append(Double.toString(_rightFollowerGains.KP));
            _sb.append(" | ");
            _sb.append(Double.toString(_rightFollowerGains.KI)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_rightFollowerGains.KD)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_rightFollowerGains.KV)); 
            _sb.append(" | ");
            _sb.append(Double.toString(_rightFollowerGains.KA));

            logData.AddData("RgtFollower:Gains", _sb.toString());
          
            Segment currentRightSegment = _rightFollower.getSegment();
            logData.AddData("RgtFollower:SegmentPos", Double.toString(currentRightSegment.position));
            logData.AddData("RgtFollower:SegmentVel", Double.toString(currentRightSegment.velocity));
            logData.AddData("RgtFollower:SegmentAccel", Double.toString(currentRightSegment.acceleration));
        }
    }

    @Override
    public void updateDashboard() {

    }
}