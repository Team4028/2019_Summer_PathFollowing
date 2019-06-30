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

// Command used during Talon Velocity Loop Tuning
public class DriveClosedLoopFixedVelocity extends Command {

  // working variables
  private Chassis _chassis = Robot._Chassis;

  private String _markerName;
  private int _pidSlotIndex;
  private long _startTimeInMS;
  private long _runTimeInMS;
  private double _leftTargetVelocityInInchesPerSec;
  private double _rightTargetVelocityInInchesPerSec;
  
  // ======================================================================================
  // constructor  
  // ======================================================================================
  public DriveClosedLoopFixedVelocity(String markerName, int pidSlotIndex, long runTimeInSec, double leftTargetVelocityInInchesPerSec, double rightTargetVelocityInInchesPerSec) 
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
    _chassis.setActivePIDProfileSlot(_pidSlotIndex);

    // set marker in log file records so we can tell when the command started in telop mode
    if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
    {
      Robot._DataLogger.setMarker(_markerName);
    }

    _chassis.zeroSensors();
    Robot._NavX.zeroYaw();
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
    // exit criteria is elapsed time
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
    
    if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
    {
      Robot._DataLogger.clearMarker();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _chassis.stop(true);

    if(Robot._DataLogger != null && Robot._DataLogger.get_isLoggingEnabled())
    {
      Robot._DataLogger.clearMarker();
    }
  }
}
