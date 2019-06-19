/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import static jaci.pathfinder.Pathfinder.boundHalfDegrees;
import static jaci.pathfinder.Pathfinder.r2d;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.EncoderFollowerPIDGainsBE;
import frc.robot.util.LogDataBE;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path
public class DriveFollowPathClosedLoop extends Command implements IBeakSquadDataPublisher {

    private Chassis _chassis = Robot._Chassis;
    private GyroNavX _navX = Robot._navX;

    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    private Notifier _notifier = new Notifier(this::followPath);

    // The starting positions of the left and right sides of the drivetrain
    private double _leftStartingDistance;
    private double _rightStartingDistance;

    private double _period;

    /*
    Equation: output = (kP * error) + (kD * diff(error)/dt) + (kV * vel) + (kA * accel)

    kP -> Proportional gain. Units: %/m. I typically start off with this at around 0.8-1.2, using a standard AM drivebase.

    kI -> pathfinder ignores kI

    kD -> Derivative gain. Units: %/(m/s). I’ll typically only tune this if tracking is bad, 
          so usually I’ll keep it at 0 unless I have a reason to change it. 
          You can think of it like a way to increase the value that the kV term puts out, 
          which can help in the lower velocity ranges if your acceleration is bad.

    kV -> Velocity Feed-forward gain. Units: %/(m/s). This should be max_%/max_vel (i.e. 1 / max_vel). 
          This is used to give the loop some kind of knowledge about what its velocity should be. 
          As Oblarg mentioned, in other implementations (i.e. Talon SRX) this will be calculated differently.

    kA -> Acceleration Feed-forward gain. Units: %/(m/s/s). 
          I typically leave this value at 0, but you can adjust it if you’re unhappy with the speed of your robot 
          and need more power in the acceleration phase(s) of the loop. This value can also be pretty dangerous if you tune it too high.
    */
    private final static EncoderFollowerPIDGainsBE _leftFollowerGains 
            = new EncoderFollowerPIDGainsBE(1.0, 0.0, 1.0, 0.0);

    private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
            = new EncoderFollowerPIDGainsBE(1.0, 0.0, 1.0, 0.0);

    // constructor
    public DriveFollowPathClosedLoop(String pathName) {
        requires(_chassis);

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
    }

    @Override
    protected void initialize() {
        // Set the starting positions of the left and right sides of the drivetrain
        _leftStartingDistance = _chassis.getLeftChassisPositionInInches();
        _rightStartingDistance = _chassis.getRightChassisPositionInInches();

        // Make sure we're starting at the beginning of the path
        _leftFollower.reset();
        _rightFollower.reset();

        _chassis.setActivePIDConstantsSlot(1);

        // Start running the path
        _notifier.startPeriodic(_period);
    }

    @Override
    protected boolean isFinished() {
        return _leftFollower.isFinished() || _rightFollower.isFinished();
    }

    @Override
    protected void end() {
        _notifier.stop();
    }

    @Override
    protected void interrupted() {
        _notifier.stop();
    }

    // name of this path
    // https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java
    // https://wpilib.screenstepslive.com/s/currentCS/m/84338/l/1021631-integrating-path-following-into-a-robot-program
    // https://www.chiefdelphi.com/t/tuning-pathfinder-pid-talon-motion-profiling-magic-etc/162516
    private void importPath(String pathName) {
        try {
            // Read the path files from the file system
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".right");
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory("output/" + pathName + ".left");

            // Set the two paths in the followers
            _leftFollower = new DistanceFollower(leftTrajectory);
            _rightFollower = new DistanceFollower(rightTrajectory);

            _period = leftTrajectory.get(0).dt;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void followPath() {
        // If either of the followers have finished their paths we need to stop the
        // notifier
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            _notifier.stop();
            _chassis.stop(false);
            return;
        }

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
          String leftGains = Double.toString(_leftFollowerGains.KP) + " | " + 
                              Double.toString(_leftFollowerGains.KI) + " | " + 
                              Double.toString(_leftFollowerGains.KD) + " | " + 
                              Double.toString(_leftFollowerGains.KV) + " | " + 
                              Double.toString(_leftFollowerGains.KA);
          logData.AddData("LeftFollower:Gains", leftGains);
    
          Segment currentLeftSegment = _leftFollower.getSegment();
          logData.AddData("LeftFollower:SegmentPos", Double.toString(currentLeftSegment.position));
          logData.AddData("LeftFollower:SegmentVel", Double.toString(currentLeftSegment.velocity));
          logData.AddData("LeftFollower:SegmentAccel", Double.toString(currentLeftSegment.acceleration));
        }
    
        if (!_rightFollower.isFinished()) 
        {
          String rightGains = Double.toString(_rightFollowerGains.KP) + " | " + 
                              Double.toString(_rightFollowerGains.KI) + " | " + 
                              Double.toString(_rightFollowerGains.KD) + " | " + 
                              Double.toString(_rightFollowerGains.KV) + " | " + 
                              Double.toString(_rightFollowerGains.KA);
          logData.AddData("RgtFollower:Gains", rightGains);
          
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