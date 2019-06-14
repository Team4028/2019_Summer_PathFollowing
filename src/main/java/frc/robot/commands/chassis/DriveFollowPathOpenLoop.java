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
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.EncoderFollowerPIDGainsBE;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

// Command to Drive following a Path
public class DriveFollowPathOpenLoop extends Command {

  private Chassis _chassis = Robot._Chassis;
  private GyroNavX _navX = Robot._navX;

  private DistanceFollower _leftFollower;
  private DistanceFollower _rightFollower;

  private Notifier _notifier = new Notifier(this::followPath);

  // The starting positions of the left and right sides of the drivetrain
  private double _leftStartingDistance;
  private double _rightStartingDistance;

  private double _period;

  private final static EncoderFollowerPIDGainsBE _leftFollowerGains 
        = new EncoderFollowerPIDGainsBE(0.15, 0.0, 1.0 / 70, 0.0);
        //  = new EncoderFollowerPIDGainsBE(1.75, 0.25, 1.0 / Chassis.MAX_VEL_IN_PER_SEC, 0);

  private final static EncoderFollowerPIDGainsBE _rightFollowerGains 
        = new EncoderFollowerPIDGainsBE(0.15, 0.0, 1.0 / 70, 0.0);
        //  = new EncoderFollowerPIDGainsBE(1.75, 0.25, 1.0 / Chassis.MAX_VEL_IN_PER_SEC, 0);

  // constructor
  public DriveFollowPathOpenLoop(String pathName) {
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

    //Make sure we're starting at the beginning of the path
    _leftFollower.reset();
    _rightFollower.reset();

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
    // If either of the followers have finished their paths we need to stop the notifier
    if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
      _notifier.stop();
      return;
    } 
    
    // Get the left and right power output from the distance calculator
    double left_speed = _leftFollower.calculate(_chassis.getLeftChassisPositionInInches() - _leftStartingDistance);
    double right_speed = _rightFollower.calculate(_chassis.getRightChassisPositionInInches() - _rightStartingDistance);

    // Calculate any correction we need based on the current and desired heading
    double heading = _navX.getPathfinderYaw();
    double desired_heading = r2d(_leftFollower.getHeading());
    double heading_difference = boundHalfDegrees(desired_heading - heading);
    double turn =  0.8 * (-1.0/80.0) * heading_difference;

    // Send the % outputs to the drivetrain
    _chassis.setOpenLoopVelocityCmd(left_speed + turn, right_speed - turn);
  }
}