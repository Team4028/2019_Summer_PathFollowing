package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController.Thumbstick;
import frc.robot.ux.OI;

public class DriveWithControllers extends Command {

  private Chassis _chassis = Robot._Chassis;
  private OI _oi = OI.getInstance();

  public DriveWithControllers() 
  {
    requires(_chassis);
    
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    _chassis.arcadeDrive(_oi.getThrottleCmd(), _oi.getTurnCmd());
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
