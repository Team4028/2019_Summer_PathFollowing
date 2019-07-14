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
import frc.robot.entities.VelocityCmdBE;
import frc.robot.util.BeakDistanceFollower;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.PoseEstimation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path using Open Loop on the TalonSRX and Notifier for the RoboRio loop
// This version uses a custom implementation of Jaci's DistanceFollower called BeakDistanceFollower
public class DriveFollowPathOpenLoopV3 extends Command implements IBeakSquadDataPublisher {

    // working variables
    private Chassis _chassis = Robot._Chassis;
    private GyroNavX _navX = Robot._NavX;

    private BeakDistanceFollower _leftFollower;
    private BeakDistanceFollower _rightFollower;

    // create notifier that will drive the timing loops
    private Notifier _notifier = new Notifier(this::followPath);

    // The starting positions of the left and right sides of the drivetrain
    private double _leftStartingDistance = 0;
    private double _rightStartingDistance = 0;

    private VelocityCmdAdjBE _lastTurnAdjustment;
    private VelocityCmdBE _lastLeftMtrCmd;
    private VelocityCmdBE _lastRgtMtrCmd;

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

    private enum TurnCmdChg
    {
        NONE,
        INCREASE,
        INCR_CAP,
        DECREASE,
        PxErr
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
     * acceleration is bad.  (Is Postion Error Growing?)
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
                            = new EncoderFollowerPIDGainsBE(0.05, 0.02, 1.0/130.0, 0.01);

    private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
                            = new EncoderFollowerPIDGainsBE(0.05, 0.02, 1.0/130.0, 0.01);

    // This constant multiplies the effect of the heading 
    // compensation on the motor output (Original equation assumes
    // open loop [-1 - 1], so this compensates for closed loop)
    private static final double KH = 0.15; 

    //  robot drives have a voltage "dead-zone" around zero within which the torque generated 
    //  by the motors is insufficient to overcome frictional losses in the drive. 
    private static final double V_INTERCEPT = 0.08;
    // ======================================================================================
    // constructor
    // ======================================================================================
    public DriveFollowPathOpenLoopV3(String pathName, Runnable loggingMethodDelegate) {
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
        _chassis.stop(true);

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
        _chassis.stop(true);

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
            _leftFollower = new BeakDistanceFollower(leftTrajectory);
            _rightFollower = new BeakDistanceFollower(rightTrajectory);

            _loopPeriodInMS = leftTrajectory.get(0).dt;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void followPath() {

        if(!isFinished()) {
            // Get the left and right power output from the distance calculator
            /*
                double calculated_value =
                        kp * error +                                    // Proportional
                        kd * ((error - last_error) / seg.dt) +          // Derivative
                        kv * seg.velocity + 
                        ka * seg.acceleration);    // V and A Terms
            */
            double leftDistanceCovered = _chassis.getLeftChassisPositionInInches() - _leftStartingDistance;
            VelocityCmdBE leftMtrCmd = _leftFollower.calculate(leftDistanceCovered);

            double rightDistanceCovered = _chassis.getRightChassisPositionInInches() - _rightStartingDistance;
            VelocityCmdBE rgtMtrCmd = _rightFollower.calculate(rightDistanceCovered);

            // if we have a non-zero command add the V Intercept (0.025 is the command deadband we use)
            leftMtrCmd.set_mtrStictionAdjCmd(V_INTERCEPT);
            rgtMtrCmd.set_mtrStictionAdjCmd(V_INTERCEPT);

            // Calculate any turn correction we need based on the current and desired heading
            double actualHeading = _navX.getHeadingInDegrees();         // CW is +
            double targetHeading = r2d(_leftFollower.getTargetHeadingInRadians());     // CW is +  
            double headingError = Pathfinder.boundHalfDegrees(targetHeading - actualHeading);   // + error means turn RIGHT
            VelocityCmdAdjBE turnAdjustment = CalcTurnAdjustment2(headingError, _lastTurnAdjustment);

            // set turn adjustment amount
            leftMtrCmd.set_mtrTurnAdjCmd(turnAdjustment.LeftMtrCmdTurnAdj);
            rgtMtrCmd.set_mtrTurnAdjCmd(turnAdjustment.RgtMtrCmdTurnAdj);

            // Send the % output motor cmd to the drivetrain (1 will be+, 1 will be -)
            _chassis.setOpenLoopVelocityCmd(leftMtrCmd.get_FinalMtrCmd(), rgtMtrCmd.get_FinalMtrCmd());
        
            _lastTurnAdjustment = turnAdjustment;
            _lastLeftMtrCmd = leftMtrCmd;
            _lastRgtMtrCmd = rgtMtrCmd;

            // if a logging method delegate was passed in, call it 
            if (_loggingMethodDelegate != null && !isFinished()) {
                _loggingMethodDelegate.run();
            }
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
    private VelocityCmdAdjBE CalcTurnAdjustment2(double currentHeadingErrorInDegrees, VelocityCmdAdjBE previousTurnAdj) {

        // constants
        final double ERROR_DEADBAND_IN_DEGREES = 0.0;
        final double TURN_MAX_ADJUSTMENT_IN_PERCENT_VBUS = 0.25;    // so min 10 cycles to reach adj value

        // working variables
        TurnDirection turnDirection;
        TurnCmdChg turnCmdChg = TurnCmdChg.PxErr;

        double leftTurnAdj = 0;
        double rgtTurnAdj = 0;

        // currentHeadingErrorInDegrees = Target - Actual
        //  <actual> ------  <target> if currentHeading Error > 0 :: too far left => turn right
        if (currentHeadingErrorInDegrees > ERROR_DEADBAND_IN_DEGREES) {
            turnDirection = TurnDirection.RIGHT;
        }
        //  <target> ------  <actual> if currentHeading Error < 0 :: too far right => turn left
        else if (currentHeadingErrorInDegrees < (-1.0 * ERROR_DEADBAND_IN_DEGREES)) {
            turnDirection = TurnDirection.LEFT;
        }
        //  within +/- deadband of target
        else turnDirection = TurnDirection.NONE;
 
        double turnAdj =  Math.abs(KH * currentHeadingErrorInDegrees);

        // =========================
        // calc new turn adjustments
        // =========================
        if (turnDirection == TurnDirection.LEFT) {
            leftTurnAdj = -turnAdj;
            rgtTurnAdj = turnAdj;
        }
        else if (turnDirection == TurnDirection.RIGHT) {
            leftTurnAdj = turnAdj;
            rgtTurnAdj = -turnAdj;
        }

        return new VelocityCmdAdjBE(leftTurnAdj, rgtTurnAdj, currentHeadingErrorInDegrees, turnDirection.toString(), turnCmdChg.toString());
    };

    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggngEnabled) {

        //if(!_leftFollower.isFinished() && !_rightFollower.isFinished()) {

            logData.AddData("Follower:PathName", _pathName.toString());
            logData.AddData("Follower:HeadingErrorInDeg", Double.toString(GeneralUtilities.roundDouble(_lastTurnAdjustment.HeadingErrorInDegrees, 3)));
            logData.AddData("Follower:TurnDirection", _lastTurnAdjustment.TurnDirection);
            logData.AddData("Follower:TurnCmdChg", _lastTurnAdjustment.TurnCmdChg);

            // calc new robot pose
            RobotPoseBE currentTargetRobotPose = PoseEstimation.EstimateNewPoseV1(_previousTargetRobotPose,
                                                                                    _lastLeftMtrCmd.CurrentSegment.position,
                                                                                    _lastRgtMtrCmd.CurrentSegment.position,
                                                                                    Math.toDegrees(_lastLeftMtrCmd.CurrentSegment.heading));

            // *********************************
            // *********** LEFT SIDE ***********
            // *********************************

            // use stringbuilder instead of concat for perf
            _sb.setLength(0);
            _sb.append("kP: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KP, 3)));
            _sb.append(" |kI: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KI, 3)));
            _sb.append(" |kD: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KD, 4)));
            _sb.append(" |kV: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KV, 5)));
            _sb.append(" |kA: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KA, 3)));
            _sb.append(" |kH: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            _sb.append(" |VI: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(V_INTERCEPT, 3)));
            logData.AddData("LeftFollower:Gains", _sb.toString());
    
            // log segment target data
            logData.AddData("LeftFollower:SegmentPos", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.CurrentSegment.position, 1)));
            logData.AddData("LeftFollower:SegmentVel", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.CurrentSegment.velocity, 2)));
            logData.AddData("LeftFollower:SegmentAccel", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.CurrentSegment.acceleration, 2)));
            logData.AddData("LeftFollower:Heading", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.CurrentSegment.heading, 3)));

            logData.AddData("LeftFollower:PosErrInInches", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.PositionErrorInInches, 2)));

            logData.AddData("LeftFollower:PCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.MtrPCmd, 3)));
            logData.AddData("LeftFollower:DCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.MtrDCmd, 3)));
            logData.AddData("LeftFollower:VCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.MtrVCmd, 3)));
            logData.AddData("LeftFollower:ACmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.MtrACmd, 3)));
            logData.AddData("LeftFollower:RawBaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.get_RawBaseMtrCmd(), 3)));
            logData.AddData("LeftFollower:AdjBaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.get_AdjBaseMtrCmd(), 3)));
            logData.AddData("LeftFollower:TurnAdj", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.get_mtrTurnAdjCmd(), 3)));
            logData.AddData("LeftFollower:FinalMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastLeftMtrCmd.get_FinalMtrCmd(), 3)));

            // log calculated target position relative to start
            logData.AddData("LeftFollower:PoseX", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.LeftXInInches, 1)));
            logData.AddData("LeftFollower:PoseY", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.LeftYInInches, 1)));

            // *********************************
            // *********** RIGHT SIDE **********
            // *********************************
            _sb.setLength(0);
            _sb.append("kP: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KP, 3)));
            _sb.append(" |kI: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KI, 3)));
            _sb.append(" |kD: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KD, 4)));
            _sb.append(" |kV: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KV, 5)));
            _sb.append(" |kA: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KA, 3)));
            _sb.append(" |kH: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            _sb.append(" |VI: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(V_INTERCEPT, 3)));
            logData.AddData("RgtFollower:Gains", _sb.toString());
          
            // log segment target data
            logData.AddData("RgtFollower:SegmentPos", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.CurrentSegment.position, 1)));
            logData.AddData("RgtFollower:SegmentVel", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.CurrentSegment.velocity, 2)));
            logData.AddData("RgtFollower:SegmentAccel", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.CurrentSegment.acceleration, 2)));
            logData.AddData("RgtFollower:Heading", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.CurrentSegment.heading, 3)));

            logData.AddData("RgtFollower:PosErrInInches", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.PositionErrorInInches, 2)));

            logData.AddData("RgtFollower:PCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.MtrPCmd, 3)));
            logData.AddData("RgtFollower:DCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.MtrDCmd, 3)));
            logData.AddData("RgtFollower:VCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.MtrVCmd, 3)));
            logData.AddData("RgtFollower:ACmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.MtrACmd, 3)));
            logData.AddData("RgtFollower:RawBaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.get_RawBaseMtrCmd(), 3)));
            logData.AddData("RgtFollower:AdjBaseMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.get_AdjBaseMtrCmd(), 3)));
            logData.AddData("RgtFollower:TurnAdj", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.get_mtrTurnAdjCmd(), 3)));
            logData.AddData("RgtFollower:FinalMtrCmd", Double.toString(GeneralUtilities.roundDouble(_lastRgtMtrCmd.get_FinalMtrCmd(), 3)));

            // log calculated target position relative to start
            logData.AddData("RgtFollower:PoseX", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.RightXInInches, 1)));
            logData.AddData("RgtFollower:PoseY", Double.toString(GeneralUtilities.roundDouble(currentTargetRobotPose.RightYInInches, 1)));

            // snapshot values for next loop cycle
            _previousTargetRobotPose = currentTargetRobotPose;
        //}
    }

    @Override
    public void updateDashboard() 
    {
        SmartDashboard.putString("PathFollower:PathName", _pathName);
    }
}