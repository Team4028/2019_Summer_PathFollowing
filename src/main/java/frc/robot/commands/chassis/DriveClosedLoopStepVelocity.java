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

public class DriveClosedLoopStepVelocity extends Command 
{

  // working variables
  private Chassis _chassis = Robot._Chassis;

  private String _markerName;
  private double _ipsStep;
  private double _currentIPSCmd;
  private long _startTimeInMS;
  private long _actualTotalRunTimeInMS;
  private long _targetTotalRunTimeInMS;
  private long _thisStepStartTimeInMS;
  private int _pidSlotIndex;

  public final long RUN_EACH_STEP_IN_MS = 250;

  public DriveClosedLoopStepVelocity(int targetTotalRunTimeInSec, double ipsStep, int pidSlotIndex) 
  {
    // Use requires() here to declare subsystem dependencies
    requires(_chassis);
    setInterruptible(true);

    _targetTotalRunTimeInMS = targetTotalRunTimeInSec * 1000;
    _ipsStep = ipsStep;
    _pidSlotIndex = pidSlotIndex;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    // init runtime timer
    _startTimeInMS = RobotController.getFPGATime() / 1000;
    _thisStepStartTimeInMS = _startTimeInMS;

    // set marker in log file records so we can tell when the command started in telop mode
    if(Robot._DataLogger != null)
    {
      Robot._DataLogger.setMarker(_markerName);
    }

    // set correct pid constants to use
    _chassis.setActivePIDConstantsSlot(_pidSlotIndex);

    _chassis.zeroSensors();
    Robot._NavX.zeroYaw();

    System.out.println(">>>>>>>>> Step command started! <<<<<<<<<<");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    long currentTimeInMS = (RobotController.getFPGATime() / 1000);

    // calc total elapsed time
    _actualTotalRunTimeInMS = currentTimeInMS - _startTimeInMS;

    // calc how long we have been running at this speed
    long thisStepRunTimeInMS = currentTimeInMS - _thisStepStartTimeInMS;

    // have we been running for at least the target time per step?
    if(thisStepRunTimeInMS >= RUN_EACH_STEP_IN_MS)
    {
       // have been running for more than 1/2 of the total time?
      if(_actualTotalRunTimeInMS < (_targetTotalRunTimeInMS / 2))
      {
        // step vbus cmd up
        _currentIPSCmd = _currentIPSCmd + _ipsStep;
      }
      else
      {
        // step vbus cmd down
        _currentIPSCmd = _currentIPSCmd - _ipsStep;
      }

      // snapshot when this step starting running
      _thisStepStartTimeInMS = currentTimeInMS;

      System.out.println(Double.toString(_currentIPSCmd));
    }

    // set marker in log file records so we can tell when the command started in telop mode
    if(Robot._DataLogger != null)
    {
      Robot._DataLogger.setMarker(Double.toString(_currentIPSCmd));
    }

    _chassis.setClosedLoopVelocityCmd(_currentIPSCmd, _currentIPSCmd);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(_actualTotalRunTimeInMS >= _targetTotalRunTimeInMS)  
    { 
      System.out.println(">>>>>>>>> Step command complete! <<<<<<<<<<<");
      return true;
    }
    else return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _chassis.stop(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _chassis.stop(false);
  }
}
