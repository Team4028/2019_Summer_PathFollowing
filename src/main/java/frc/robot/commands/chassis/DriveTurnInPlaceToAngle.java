/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;

// ref: https://github.com/frc-4931/2018-Robot/blob/master/src/org/usfirst/frc/team4931/robot/commands/TurnToAngle.java
public class DriveTurnInPlaceToAngle extends Command {

  // working variables
  private Chassis _chassis = Robot._Chassis;
  private GyroNavX _navX = Robot._NavX;

  private static final double RAMP_UP_THRESHOLD_DISTANCE = 20;
  private static final double RAMP_DOWN_THRESHOLD_DISTANCE = 45;
  private static final double START_SPEED = 0.4;
  private static final double END_SPEED = 0.3;
  private static final double MAX_SPEED = 0.6;
  private static final double TARGET_ANGLE_ERROR_THRESHOLD = 0.5;
  private double _targetSpeed;
  private double _targetAngle;
  private double _startAngle;


  public DriveTurnInPlaceToAngle(double targetSpeed, double targetAngle) {
    // Use requires() here to declare subsystem dependencies
    requires(_chassis);
    setInterruptible(true);

    this._targetSpeed = Math.min(targetSpeed, MAX_SPEED);
    this._targetAngle = targetAngle;

    System.out.println("Starting Speed: " + targetSpeed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _startAngle = _navX.getHeadingInDegrees();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentAngle = _navX.getHeadingInDegrees() - _startAngle;
    double calcSpeed = ramp(Math.abs(currentAngle));

    if (_targetSpeed > 0) {
      _chassis.setOpenLoopVelocityCmd(calcSpeed, -calcSpeed);
    } else {
      _chassis.setOpenLoopVelocityCmd(-calcSpeed, calcSpeed);
    }

    System.out.println("Angle: " + currentAngle + "\n" + "Speed: " + calcSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(_navX.getHeadingInDegrees() - Math.abs(_targetAngle)) < TARGET_ANGLE_ERROR_THRESHOLD);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _chassis.stop(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _chassis.stop(true);
  }

  /**
   * Calculates a speed from minSpeed to maxSpeed, with a ramp up and down based on thresholdDistance. All values assumed non-negative.
   */
  private double ramp(double currentAngle) {

    return calculateSpeed(currentAngle, 
                          _targetAngle, 
                          RAMP_UP_THRESHOLD_DISTANCE,
                          RAMP_DOWN_THRESHOLD_DISTANCE, 
                          Math.abs(_targetSpeed), 
                          START_SPEED, 
                          END_SPEED);
  }

  private double calculateSpeed(double current, double target, 
                                double rampUpThreshold, double rampDownThreshold, 
                                double maxSpeed, double startSpeed, double endSpeed) {

    return Math.min(Math.min(Math.pow(current / rampUpThreshold, 0.75), 1) * (maxSpeed - startSpeed)
                  + startSpeed,
              Math.min(Math.pow(Math.max((target - current) / rampDownThreshold, 0), 1.5), 1) * (maxSpeed
                  - endSpeed)
                  + endSpeed);
  }
}
