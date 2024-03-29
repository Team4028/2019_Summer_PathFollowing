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
import frc.robot.util.GeneralUtilities;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path using Velocity Closed Loop on the TalonSRX and Notifier for the RoboRio loop
public class DriveFollowPathClosedLoop extends Command implements IBeakSquadDataPublisher {

    // working variables
    private Chassis _chassis = Robot._Chassis;
    private GyroNavX _navX = Robot._NavX;

    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    //Graphing Paths Utility Varibales
    private double _leftXCoord;
    private double _leftYCoord;
    private double _leftLastPosition = 0;
    private double _leftLastXCoord = 0;
    private double _leftLastYCoord = 0.0 + Chassis.TRACK_WIDTH_INCHES / 2.0;    // distance from Robot Centerline

    private double _rightXCoord;
    private double _rightYCoord;
    private double _rightLastPosition = 0;
    private double _rightLastXCoord = 0;
    private double _rightLastYCoord = 0.0 - Chassis.TRACK_WIDTH_INCHES / 2.0;   // distance from Robot Centerline

    // create notifier that will 
    private Notifier _notifier = new Notifier(this::followPath);

    // The starting positions of the left and right sides of the drivetrain
    private double _leftStartingDistance = 0;
    private double _rightStartingDistance = 0;

    private String _pathName = "";
    private Runnable _loggingMethodDelegate;
    private double _loopPeriodInMS = 0;
  
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
                            = new EncoderFollowerPIDGainsBE(1.4, 0.0, 1.0, 0.0);

    private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
                            = new EncoderFollowerPIDGainsBE(1.4, 0.0, 1.0, 0.0);

    // This constant multiplies the effect of the heading 
    // compensation on the motor output (Original equation assumes
    // open loop [-1 - 1], so this compensates for closed loop)
    private static final double KH = 5.0; 

    //  robot drives have a voltage "dead-zone" around zero within which the torque generated 
    //  by the motors is insufficient to overcome frictional losses in the drive. 
    private static final double V_INTERCEPT = 4.0;
    // ======================================================================================
    // constructor
    // ======================================================================================
    public DriveFollowPathClosedLoop(String pathName, Runnable loggingMethodDelegate) {
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

        // determine what chassis pid constants to use
        int chassisPidConstant = Chassis.PID_PROFILE_SLOT_IDX_HS;   // todo: determine this dynamically

        // set chassis pid constants
        _chassis.setActivePIDProfileSlot(chassisPidConstant);

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

        // Get the left and right power output from the distance calculator
        /*
            double calculated_value =
            kp * error +                                    // Proportional
            kd * ((error - last_error) / seg.dt) +          // Derivative
            (kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        */
        double left_speed = 
                _leftFollower.calculate(_chassis.getLeftChassisPositionInInches() - _leftStartingDistance);
        double right_speed = 
                _rightFollower.calculate(_chassis.getRightChassisPositionInInches() - _rightStartingDistance);

        // if we have a non-zero command add the V Intercept 
        if(left_speed > 0.1) {
            left_speed = left_speed + V_INTERCEPT;
        }
        else if(left_speed < -0.1) {
            left_speed = left_speed - V_INTERCEPT;
        }

        if(right_speed > 0.1) {
            right_speed = right_speed + V_INTERCEPT;
        }
        else if(right_speed < -0.1) {
            right_speed = right_speed - V_INTERCEPT;
        }

        // Calculate any correction we need based on the current and desired heading
        double heading = _navX.getHeadingInDegrees();
        double desired_heading = r2d(_leftFollower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn = KH * 0.8 * (-1.0 / 80.0) * heading_difference;

        // Send the IPS target velocities to the drivetrain
        _chassis.setClosedLoopVelocityCmd(left_speed - turn, right_speed + turn);
    
        // if a logging method delegate was passed in, call it 
        if (_loggingMethodDelegate != null) {
            _loggingMethodDelegate.run();
        }
    }

    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggngEnabled) {
        // use stringbuilder instead of concat for perf
        if (!_leftFollower.isFinished()) {

            _sb.setLength(0);
            _sb.append("p: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KP, 3)));
            _sb.append(" |i: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KI, 3)));
            _sb.append(" |d: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KD, 3)));
            _sb.append(" |v: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KV, 3)));
            _sb.append(" |a: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_leftFollowerGains.KA, 3)));
            _sb.append(" |h: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            logData.AddData("LeftFollower:Gains", _sb.toString());
    
            // log segment target data
            Segment currentLeftSegment = _leftFollower.getSegment();
            logData.AddData("LeftFollower:SegmentPos", Double.toString(currentLeftSegment.position));
            logData.AddData("LeftFollower:SegmentVel", Double.toString(currentLeftSegment.velocity));
            logData.AddData("LeftFollower:SegmentAccel", Double.toString(currentLeftSegment.acceleration));
            logData.AddData("LeftFollower:Heading", Double.toString(currentLeftSegment.heading));
            
            // log calculated target position relative to start
            double leftDPSegment = currentLeftSegment.position - _leftLastPosition;
            double leftDXSegment = leftDPSegment * Math.cos(currentLeftSegment.heading);
            double leftDYSegment = -leftDPSegment * Math.sin(currentLeftSegment.heading);

            _leftXCoord = _leftLastXCoord + leftDXSegment;
            _leftYCoord = _leftLastYCoord + leftDYSegment;

            logData.AddData("LeftFollower:X", Double.toString(GeneralUtilities.roundDouble(_leftXCoord, 1)));
            logData.AddData("LeftFollower:Y", Double.toString(GeneralUtilities.roundDouble(_leftYCoord, 1)));

            // snapshot values for next loop cycle
            _leftLastPosition = currentLeftSegment.position;
            _leftLastXCoord = _leftXCoord;
            _leftLastYCoord = _leftYCoord;
        }
    
        if (!_rightFollower.isFinished()) {

            _sb.setLength(0);
            _sb.append("p: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KP, 3)));
            _sb.append(" |i: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KI, 3)));
            _sb.append(" |d: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KD, 3)));
            _sb.append(" |v: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KV, 3)));
            _sb.append(" |a: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(_rightFollowerGains.KA, 3)));
            _sb.append(" |h: ");
            _sb.append(Double.toString(GeneralUtilities.roundDouble(KH, 3)));
            logData.AddData("RgtFollower:Gains", _sb.toString());
          
            // log segment target data
            Segment currentRightSegment = _rightFollower.getSegment();
            logData.AddData("RgtFollower:SegmentPos", Double.toString(currentRightSegment.position));
            logData.AddData("RgtFollower:SegmentVel", Double.toString(currentRightSegment.velocity));
            logData.AddData("RgtFollower:SegmentAccel", Double.toString(currentRightSegment.acceleration));
            logData.AddData("RGTFollower:Heading", Double.toString(currentRightSegment.heading));

            // log calculated target position relative to start
            double rightDPSegment = currentRightSegment.position - _rightLastPosition;
            double rightDXSegment = rightDPSegment * Math.cos(currentRightSegment.heading);
            double rightDYSegment = -rightDPSegment * Math.sin(currentRightSegment.heading);

            _rightXCoord = _rightLastXCoord + rightDXSegment;
            _rightYCoord = _rightLastYCoord + rightDYSegment;

            logData.AddData("RgtFollower:X", Double.toString(_rightXCoord));
            logData.AddData("RgtFollower:Y", Double.toString(_rightYCoord));

            // snapshot values for next loop cycle
            _rightLastPosition = currentRightSegment.position;
            _rightLastXCoord = _rightXCoord;
            _rightLastYCoord = _rightYCoord;
        }
    }

    @Override
    public void updateDashboard() 
    {
        SmartDashboard.putString("PathFollower:PathName", _pathName);
    }
}