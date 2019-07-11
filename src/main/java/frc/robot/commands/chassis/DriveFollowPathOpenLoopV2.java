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
import frc.robot.entities.VelocityCmdAdjBE;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.PoseEstimation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path using Open Loop on the TalonSRX and Notifier for the RoboRio loop
public class DriveFollowPathOpenLoopV2 extends Command implements IBeakSquadDataPublisher {

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

    private VelocityCmdAdjBE _lastTurnAdjustment;

    private String _pathName = "";
    private Runnable _loggingMethodDelegate;
    private double _loopPeriodInMS = 0;

    private RobotPoseBE _previousTargetRobotPose;

    // init stringbuilder size to improve perf
    private StringBuilder _sb = new StringBuilder(250);

    private enum TurnDirection
    {
        NONE,
        LEFT,
        RIGHT
    }

    private enum TurnChgType
    {
        NONE,
        INCREASE,
        DECREASE
    }

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
    public DriveFollowPathOpenLoopV2(String pathName, Runnable loggingMethodDelegate) {
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
        _lastTurnAdjustment = VelocityCmdAdjBE.init();
        _previousTargetRobotPose = RobotPoseBE.init();

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
        double actualHeading = _navX.getHeadingInDegrees();         // CW is +
        double targetHeading = r2d(_leftFollower.getHeading());     // CW is +  
        double headingError = Pathfinder.boundHalfDegrees(targetHeading - actualHeading);   // + error means turn RIGHT

        VelocityCmdAdjBE turnAdjustment = CalcTurnAdjustment(headingError, _lastTurnAdjustment);
        turnAdjustment.LeftBaseMtrCmd = left_speed;
        turnAdjustment.RgtBaseMtrCmd = right_speed;

        // Send the % output motor cmd to the drivetrain (1 will be+, 1 will be -)
        _chassis.setOpenLoopVelocityCmd(turnAdjustment.LeftFinalMtrCmd(), turnAdjustment.RgtFinalMtrCmd());
    
        _lastTurnAdjustment = turnAdjustment;

        // if a logging method delegate was passed in, call it 
        if (_loggingMethodDelegate != null && !isFinished()) {
            _loggingMethodDelegate.run();
        }
    }

    //
    // Calculate the Turn Adjustment
    //
    // + angle is CW Turn
    //            0
    //            ^
    //    - Turn  |  + Turn
    //            |
    //  -90 <-----+------> +90
    //
    //  Our working theory is that JACI's simple turn adj calc: 
    //      double turn = KH * 0.8 * (-1.0 / 80.0) * heading_difference;
    //  will not work for some chassis wheel configs and possibly the same error
    //      will require a different adj value at different chassis speeds
    //  what we observed is that the std calc was not effecive at higher speeds and the error
    //      was increasing across the accel and decel parts of the graph
    //  this method attempts to "auto scale" the adj value until it sees the chassis respond
    //
    //  our expectation is that the turn component is never the driving part of the command, just an incremental adjustment
    //
    private VelocityCmdAdjBE CalcTurnAdjustment(double currentHeadingErrorInDegrees, VelocityCmdAdjBE previousTurnAdj) {

        // constants
        final double ERROR_DEADBAND_IN_DEGREES = 0.5;
        final double ERROR_CHG_DEADBAND_IN_DEGREES = 0.1;
        final double TURN_ADJUSTMENT_IN_PERCENT_VBUS = 0.025;
        final double TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS = 0.25;    // so min 10 cycles to reach adj value
        
        // working variables
        TurnDirection turnDirection;
        TurnChgType turnChgType;

        double lastLeftTurnAdj = 0;
        double lastRgtTurnAdj = 0;

        double leftTurnAdj = 0;
        double rgtTurnAdj = 0;

        // if currentHeading Error < 0 :: turn right, else if currentHeading Error > 0 :: turn left
        if (currentHeadingErrorInDegrees > ERROR_DEADBAND_IN_DEGREES) {
            turnDirection = TurnDirection.LEFT;
        }
        else if (currentHeadingErrorInDegrees < (-1.0 * ERROR_DEADBAND_IN_DEGREES)) {
            turnDirection = TurnDirection.RIGHT;
        }
        else turnDirection = TurnDirection.NONE;

        // if chgInHeadingError > 0 (ie error is increasing) :: incr adj, else if chgInHeadingError < 0 (ie error is decreasing):: decr adj
        boolean didErrorSignChg = !((currentHeadingErrorInDegrees < 0) == (previousTurnAdj.HeadingErrorInDegrees < 0)); 
        double chgInHeadingError = (Math.abs(currentHeadingErrorInDegrees) - Math.abs(previousTurnAdj.HeadingErrorInDegrees));
        if(didErrorSignChg) {
            turnChgType = TurnChgType.INCREASE;
            lastLeftTurnAdj = 0;
            lastRgtTurnAdj = 0;
        }
        // error is increasing
        else if (Math.abs(chgInHeadingError) >= Math.abs(ERROR_CHG_DEADBAND_IN_DEGREES)) {
            turnChgType = TurnChgType.INCREASE;
            lastLeftTurnAdj = previousTurnAdj.LeftMtrCmdTurnAdj;
            lastRgtTurnAdj =  previousTurnAdj.RgtMtrCmdTurnAdj;
        }
        // error is decreasing
        else if (Math.abs(chgInHeadingError) <= Math.abs(ERROR_CHG_DEADBAND_IN_DEGREES)) {
            turnChgType = TurnChgType.NONE; //TurnChgType.DECREASE; // ??????
            lastLeftTurnAdj = previousTurnAdj.LeftMtrCmdTurnAdj;
            lastRgtTurnAdj =  previousTurnAdj.RgtMtrCmdTurnAdj;
        }
        else {
            turnChgType = TurnChgType.NONE;
            lastLeftTurnAdj = previousTurnAdj.LeftMtrCmdTurnAdj;
            lastRgtTurnAdj =  previousTurnAdj.RgtMtrCmdTurnAdj;
        }

        // calc left turns
        if (turnDirection == TurnDirection.LEFT) {
            if (turnChgType == TurnChgType.INCREASE) {
                leftTurnAdj = lastLeftTurnAdj - TURN_ADJUSTMENT_IN_PERCENT_VBUS;
                rgtTurnAdj = lastRgtTurnAdj + TURN_ADJUSTMENT_IN_PERCENT_VBUS;
            }
            else if (turnChgType == TurnChgType.DECREASE) {
                leftTurnAdj = lastLeftTurnAdj + TURN_ADJUSTMENT_IN_PERCENT_VBUS;
                rgtTurnAdj = lastRgtTurnAdj - TURN_ADJUSTMENT_IN_PERCENT_VBUS;
            }
            else if (turnChgType == TurnChgType.NONE) {
                leftTurnAdj = lastLeftTurnAdj;
                rgtTurnAdj = lastRgtTurnAdj;
            }
        }
        // calc right turns
        else if (turnDirection == TurnDirection.RIGHT) {
            if (turnChgType == TurnChgType.INCREASE) {
                leftTurnAdj = lastLeftTurnAdj + TURN_ADJUSTMENT_IN_PERCENT_VBUS;
                rgtTurnAdj = lastRgtTurnAdj - TURN_ADJUSTMENT_IN_PERCENT_VBUS;
            }
            else if (turnChgType == TurnChgType.DECREASE) {
                leftTurnAdj = lastLeftTurnAdj - TURN_ADJUSTMENT_IN_PERCENT_VBUS;
                rgtTurnAdj = lastRgtTurnAdj + TURN_ADJUSTMENT_IN_PERCENT_VBUS;
            }
            else if (turnChgType == TurnChgType.NONE) {
                leftTurnAdj = lastLeftTurnAdj;
                rgtTurnAdj = lastRgtTurnAdj;
            }
        }
        //no turn, w/i deadband
        else if (turnDirection == TurnDirection.NONE) {
            leftTurnAdj = 0;
            rgtTurnAdj = 0;
        }

        // check max adjustment
        if (leftTurnAdj > TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS) {
            leftTurnAdj = TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS;
        }
        else if (leftTurnAdj < (-1.0 * TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS)) {
            leftTurnAdj = -1.0 * TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS;
        };

        if (rgtTurnAdj > TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS) {
            rgtTurnAdj = TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS;
        }
        else if (rgtTurnAdj < (-1.0 * TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS)) {
            rgtTurnAdj = -1.0 * TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS;
        };

        return new VelocityCmdAdjBE(leftTurnAdj, rgtTurnAdj, currentHeadingErrorInDegrees);
    };


    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggngEnabled) {

        if(!_leftFollower.isFinished() && !_rightFollower.isFinished()) {

            logData.AddData("Follower:PathName", _pathName.toString());

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
            logData.AddData("LeftFollower:BaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.LeftBaseMtrCmd, 3)));
            logData.AddData("LeftFollower:TurnAdj", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.LeftMtrCmdTurnAdj, 3)));
            logData.AddData("LeftFollower:FinalMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.LeftFinalMtrCmd(), 3)));
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
            logData.AddData("RgtFollower:BaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.RgtBaseMtrCmd, 3)));
            logData.AddData("RgtFollower:TurnAdj", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.RgtMtrCmdTurnAdj, 3)));
            logData.AddData("RgtFollower:FinalMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.RgtFinalMtrCmd(), 3)));
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