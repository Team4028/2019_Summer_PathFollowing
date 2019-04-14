package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController.Thumbstick;

public class DriveWithControllers extends Command {

  private Chassis _chassis = Robot._Chassis;
  private Thumbstick _leftThumbstick;
  private Thumbstick _rightThumbstick;
  private boolean isAuton;
  private double _throttleCmd, _turnCmd;


  public DriveWithControllers(Thumbstick leftThumbstick, Thumbstick righThumbstick) 
  {
    requires(_chassis);
    setInterruptible(true);

    _leftThumbstick = leftThumbstick;
    _rightThumbstick = righThumbstick;

    isAuton=false;
  }

  public DriveWithControllers(double throttle, double turn) {
    requires(_chassis);
    setInterruptible(true);

    _throttleCmd=throttle;
    _turnCmd=turn;

    isAuton = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(!isAuton) {
      _chassis.arcadeDrive((_leftThumbstick.getY() * 1.0), (_rightThumbstick.getX() * 1.0));
    } else {
      _chassis.arcadeDrive(_throttleCmd, _turnCmd);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    //_chassis.stop(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {}
}
