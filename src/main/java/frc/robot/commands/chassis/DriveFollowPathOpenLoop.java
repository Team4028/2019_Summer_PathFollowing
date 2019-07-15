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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.entities.EncoderFollowerPIDGainsBE;
import frc.robot.entities.LogDataBE;
import frc.robot.entities.RobotPoseBE;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.PoseEstimation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path using Open Loop on the TalonSRX and Notifier for the RoboRio loop
public class DriveFollowPathOpenLoop extends Command implements IBeakSquadDataPublisher {

    // working variables
    private Chassis _chassis = Robot._Chassis;
    private GyroNavX _navX = Robot._NavX;

    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    // create notifier that will drive the timing loops
    private Notifier _notifier = new Notifier(this::followPath);

    // The starting positions of the left and right sides of the drivetrain
    private double _leftStartingDistance = 0;
    private double _rightStartingDistance = 0;

    private double _turnAdjustment = 0;

    private String _pathName = "";
    private Runnable _loggingMethodDelegate;
    private double _loopPeriodInMS = 0;

    private RobotPoseBE _previousTargetRobotPose = RobotPoseBE.init();

    // init stringbuilder size to improve perf
    private StringBuilder _sb = new StringBuilder(250);

    /*
     * Equation: output = (kP * error) + (kD * diff(error)/dt) + (kV * vel) + (kA * accel)
     * 
     * kP -> Proportional gain. Units: %/m. I typically start off with this at
     * around 0.8-1.2, using a standard AM drivebase.
     * 
     * kI -> pathfinder ignores kI
     * 
     * kD -> Derivative gain. Units: %/(m/s). I'll typically only tune this if
     * tracking is bad, so usually I'll keep it at 0 unless I have a reason to
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
     * this value at 0, but you can adjust it if you're unhappy with the speed of
     * your robot and need more power in the acceleration phase(s) of the loop. This
     * value can also be pretty dangerous if you tune it too high.
     */
    private final static EncoderFollowerPIDGainsBE _leftFollowerGains 
                            = new EncoderFollowerPIDGainsBE(0.2, 0.0, 1.0/130.0, 0.0);

    private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
                            = new EncoderFollowerPIDGainsBE(0.2, 0.0, 1.0/130.0, 0.0);

    // This constant multiplies the effect of the heading 
    // compensation on the motor output (Original equation assumes
    // open loop [-1 - 1], so this compensates for closed loop)
    private static final double KH = 3.0; 

    //  robot drives have a voltage "dead-zone" around zero within which the torque generated 
    //  by the motors is insufficient to overcome frictional losses in the drive. 
    private static final double V_INTERCEPT = 0.08;
    // ======================================================================================
    // constructor
    // ======================================================================================
    public DriveFollowPathOpenLoop(String pathName, Runnable loggingMethodDelegate) {
        // Use requires() here to declare subsystem dependencies
        requires(_chassis);
        setInterruptible(true);

        _pathName = pathName;
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

        // Start running the path
        _notifier.startPeriodic(_loopPeriodInMS);

        System.out.println("-------------------------------------");
        System.out.println("... Running Path: " + _pathName);
        System.out.println("-------------------------------------");

        if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
        {
          Robot._DataLogger.setMarker(_pathName);
        }
    }

    @Override
    protected boolean isFinished() {
        return (_leftFollower.isFinished() || _rightFollower.isFinished());
    }

    // Commmand ended normally (typically because isFinished returns true)
    @Override
    protected void end() {
        _notifier.stop();
        _notifier.close();  // notifier was still firing events until i added this call
        _chassis.stop(false);

        if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
        {
          Robot._DataLogger.clearMarker();
        }

        System.out.println("-------------------------------------");
        System.out.println("... Completed Path: " + _pathName);
        System.out.println("-------------------------------------");
    }

    //
    @Override
    protected void interrupted() {
        _notifier.stop();
        _notifier.close();  // notifier was still firing events until i added this call
        _chassis.stop(false);

        if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
        {
          Robot._DataLogger.clearMarker();
        }

        System.out.println("-------------------------------------");
        System.out.println("... Interrupted Path: " + _pathName);
        System.out.println("-------------------------------------");
    }

    // name of this path
    // https://github.com/JacisNonsense/Pathfinder/wiki/Pathfinder-for-FRC---Java
    // https://wpilib.screenstepslive.com/s/currentCS/m/84338/l/1021631-integrating-path-following-into-a-robot-program
    // https://www.chiefdelph:i.com/t/tuning-pathfinder-pid-talon-motion-profiling-magic-etc/162516
    private void importPath(String pathName) {
        try {
            // deploy folder is /home/lvuser/deploy/paths/output
            // JACI's tool
            // Note: Bug in this version of PathFollower, generated paths are in the opposite filename
            String leftFileName =  "output/" + pathName + ".right";
            String rgtFileName = "output/" + pathName + ".left";

            // Read the path files from the file system
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory(leftFileName);
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory(rgtFileName);

            // Set the two paths in the followers
            _leftFollower = new DistanceFollower(leftTrajectory);
            _rightFollower = new DistanceFollower(rightTrajectory);

            _loopPeriodInMS = leftTrajectory.get(0).dt;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void followPath() {

        // Get the left and right power output from the distance calculator
        /*
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / seg.dt) +          // Derivative
                    kv * seg.velocity + 
                    ka * seg.acceleration);    // V and A Terms
        */
        double left_speed = 
                _leftFollower.calculate(_chassis.getLeftChassisPositionInInches() - _leftStartingDistance);
        double right_speed = 
                _rightFollower.calculate(_chassis.getRightChassisPositionInInches() - _rightStartingDistance);

        // if we have a non-zero command add the V Intercept (0.025 is the command deadband we use)
        if(left_speed > 0.025) {
            left_speed = left_speed + V_INTERCEPT;
        }
        else if(left_speed < -0.025) {
            left_speed = left_speed - V_INTERCEPT;
        }

        if(right_speed > 0.025) {
            right_speed = right_speed + V_INTERCEPT;
        }
        else if(right_speed < -0.025) {
            right_speed = right_speed - V_INTERCEPT;
        }

        // Calculate any correction we need based on the current and desired heading
        double heading = _navX.getHeadingInDegrees();   // CW is +
        double desired_heading = r2d(_leftFollower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn = KH * 0.8 * (-1.0 / 80.0) * heading_difference;

        _turnAdjustment = turn;

        // Send the % output motor cmd to the drivetrain
        _chassis.setOpenLoopVBusPercentCmd(left_speed - turn, right_speed + turn);
    
        // if a logging method delegate was passed in, call it 
        if (_loggingMethodDelegate != null && !isFinished()) {
            _loggingMethodDelegate.run();
        }
    }

    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggngEnabled) {

        if(!_leftFollower.isFinished() && !_rightFollower.isFinished()) {

            logData.AddData("Follower:PathName", _pathName.toString());
            logData.AddData("Follower:TurnAdj", Double.toString(GeneralUtilities.roundDouble(_turnAdjustment, 2)));;

            // grab current segments
            Segment currentLeftSegment = _leftFollower.getSegment();
            Segment currentRightSegment = _rightFollower.getSegment();

            // calc new robot pose
            RobotPoseBE currentTargetRobotPose = PoseEstimation.EstimateNewPoseV1(_previousTargetRobotPose,
                                                                                    currentLeftSegment.position,
                                                                                    currentRightSegment.position,
                                                                                    Math.toDegrees(currentRightSegment.heading));

            // *********************************
            // *********** LEFT SIDE ***********
            // *********************************

            // use stringbuilder instead of concat for perf
            _sb.setLength(0);
            _sb.append("p: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KP, 3)));
            _sb.append(" |i: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KI, 3)));
            _sb.append(" |d: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KD, 3)));
            _sb.append(" |v: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KV, 5)));
            _sb.append(" |a: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KA, 3)));
            _sb.append(" |h: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            _sb.append(" |vi: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(V_INTERCEPT, 3)));
            logData.AddData("LeftFollower:Gains", _sb.toString());
    
            // log segment target data
            logData.AddData("LeftFollower:SegmentPos", Double.toString(GeneralUtilities.roundDouble(currentLeftSegment.position, 1)));
            logData.AddData("LeftFollower:SegmentVel", Double.toString(GeneralUtilities.roundDouble(currentLeftSegment.velocity, 2)));
            logData.AddData("LeftFollower:SegmentAccel", Double.toString(GeneralUtilities.roundDouble(currentLeftSegment.acceleration, 2)));
            logData.AddData("LeftFollower:Heading", Double.toString(GeneralUtilities.roundDouble(currentLeftSegment.heading, 3)));
            
            // log calculated target position relative to start
            logData.AddData("LeftFollower:PoseX", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.LeftXInInches, 1)));
            logData.AddData("LeftFollower:PoseY", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.LeftYInInches, 1)));

            // *********************************
            // *********** RIGHT SIDE **********
            // *********************************
            _sb.setLength(0);
            _sb.append("p: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KP, 3)));
            _sb.append(" |i: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KI, 3)));
            _sb.append(" |d: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KD, 3)));
            _sb.append(" |v: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KV, 5)));
            _sb.append(" |a: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KA, 3)));
            _sb.append(" |h: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            _sb.append(" |vi: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(V_INTERCEPT, 3)));
            logData.AddData("RgtFollower:Gains", _sb.toString());
          
            // log segment target data
            
            logData.AddData("RgtFollower:SegmentPos", Double.toString(GeneralUtilities.roundDouble(currentRightSegment.position, 1)));
            logData.AddData("RgtFollower:SegmentVel", Double.toString(GeneralUtilities.roundDouble(currentRightSegment.velocity, 2)));
            logData.AddData("RgtFollower:SegmentAccel", Double.toString(GeneralUtilities.roundDouble(currentRightSegment.acceleration, 2)));
            logData.AddData("RgtFollower:Heading", Double.toString(GeneralUtilities.roundDouble(currentRightSegment.heading, 3)));

            // log calculated target position relative to start
            logData.AddData("RgtFollower:PoseX", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.RightXInInches, 1)));
            logData.AddData("RgtFollower:PoseY", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.RightYInInches, 1)));

            // snapshot values for next loop cycle
            _previousTargetRobotPose = currentTargetRobotPose;
        }
    }

    @Override
    public void updateDashboard() 
    {
        SmartDashboard.putString("PathFollower:PathName", _pathName);
    }
}