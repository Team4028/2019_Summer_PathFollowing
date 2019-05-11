/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.util.DataLogger;

public class DriveClosedLoopVelocity extends Command {

  private Chassis _chassis = Robot._Chassis;

  private String _markerName;
  private int _pidSlotIndex;
  private long _startTimeInMS;
  private long _runTimeInMS;
  private double _leftTargetVelocityInInchesPerSec;
  private double _rightTargetVelocityInInchesPerSec;
  
  public DriveClosedLoopVelocity(String markerName, int pidSlotIndex, long runTimeInSec, double leftTargetVelocityInInchesPerSec, double rightTargetVelocityInInchesPerSec) 
  {
    // Use requires() here to declare subsystem dependencies
    requires(_chassis);
    setInterruptible(true);

    _markerName = markerName;
    _pidSlotIndex = pidSlotIndex;
    _runTimeInMS = runTimeInSec * 1000;
    _leftTargetVelocityInInchesPerSec = leftTargetVelocityInInchesPerSec;
    _rightTargetVelocityInInchesPerSec = rightTargetVelocityInInchesPerSec;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    // init runtime timer
    //_startTimeInMS = System.currentTimeMillis();
    _startTimeInMS = RobotController.getFPGATime() / 1000;

    // set correct pid constants to use
    _chassis.setActivePIDConstantsSlot(_pidSlotIndex);

    if(Robot._DataLogger != null)
    {
      Robot._DataLogger.setMarker(_markerName);
    }

    _chassis.zeroSensors();
    Robot._navX.zeroYaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    _chassis.setClosedLoopVelocityCmd(_leftTargetVelocityInInchesPerSec, _rightTargetVelocityInInchesPerSec);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    //long currentTimeInMS = System.currentTimeMillis();
    long currentTimeInMS = RobotController.getFPGATime() / 1000;
    long elapsedTimeInMS = currentTimeInMS - _startTimeInMS;
    if(elapsedTimeInMS >= _runTimeInMS)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    _chassis.stop(true);
    if(Robot._DataLogger != null)
    {
      Robot._DataLogger.clearMarker();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _chassis.stop(true);
    if(Robot._DataLogger != null)
    {
      Robot._DataLogger.clearMarker();
    }
  }
}
