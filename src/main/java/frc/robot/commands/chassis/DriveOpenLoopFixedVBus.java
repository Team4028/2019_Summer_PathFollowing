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

// Command used to find max speed & accel
public class DriveOpenLoopFixedVBus extends Command {

  // working variables
  private Chassis _chassis = Robot._Chassis;

  private String _markerName;
  private long _startTimeInMS;
  private long _runTimeInMS;
  private double _leftTargetVBusPercent;
  private double _rightTargetVBusPercent;
  
  // ======================================================================================
  // constructor  
  // ======================================================================================
  public DriveOpenLoopFixedVBus(String markerName, long runTimeInSec, double leftTargetVBusPercent, double rightTargetVBusPercent) 
  {
    // Use requires() here to declare subsystem dependencies
    requires(_chassis);
    setInterruptible(true);

    _markerName = markerName;
    _runTimeInMS = runTimeInSec * 1000;
    _leftTargetVBusPercent = leftTargetVBusPercent;
    _rightTargetVBusPercent = rightTargetVBusPercent;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    // init runtime timer
    //_startTimeInMS = System.currentTimeMillis();
    _startTimeInMS = RobotController.getFPGATime() / 1000;

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
    _chassis.setOpenLoopVBusPercentCmd(_leftTargetVBusPercent, _rightTargetVBusPercent);
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

