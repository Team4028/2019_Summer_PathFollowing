/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.commands.chassis.DriveFollowPathClosedLoop;
import frc.robot.commands.chassis.DriveFollowPathClosedLoopV2;
import frc.robot.commands.chassis.DriveFollowPathOpenLoop;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.DataLogger;
import frc.robot.util.DataLogger2;
import frc.robot.util.GeneralUtilities;
import frc.robot.entities.LogDataBE;
import frc.robot.ux.OI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String ROBOT_NAME = "2019 PathFollowing-TomB";

  // create instance of each Subsystem (singleton)
  // Note: add each one to the outputAllToDashboard & logAllData methods below
  // ux
  private OI _oi = OI.getInstance();

  // sensors
  public static GyroNavX _navX = GyroNavX.getInstance();

  // subsystems
  public static Chassis _Chassis = Chassis.getInstance();

  // class level working variables
  public static DataLogger2 _DataLogger = null;

  private String _buildMsg = "?";
  private Command _autonomousCommand = null;
  private boolean _isNotifierRunning = false;
  private LogDataBE _logData = new LogDataBE();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    // write the overall robot dashboard info
    _buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
    SmartDashboard.putString("Robot Build", _buildMsg);

    _navX.zeroYaw();

    // https://www.chiefdelphi.com/t/improbable-java-slow-down-in-combining-doubles-and-strings/350331/17
    String forceDoubleToLoad = Double.toString(1234.56);
  }

  /********************************************************************************************
   * Autonomous Mode
   ********************************************************************************************/
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // zero sensors before auton
    _Chassis.zeroSensors();
    _navX.zeroYaw();

    // init data logging
    _DataLogger = DataLogger2.setupLogging("Auton"); 

    // setup auton command
    _autonomousCommand = new DriveFollowPathClosedLoop("Straight_v3", this::logAllData);

    // schedule the autonomous command
    if (_autonomousCommand != null) {
      _autonomousCommand.start();
      _isNotifierRunning = true;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    // run the command scheduler
    Scheduler.getInstance().run();
  }

  /********************************************************************************************
   * Telop Mode
   ********************************************************************************************/

  @Override
  public void teleopInit()
  {    
    // start honoring joysticks
    Command driveWJoyStick = new DriveWithControllers();
    driveWJoyStick.start();

    // init data logging
    _DataLogger = DataLogger2.setupLogging("Telop"); // init data logging	
  }

   /* This function is called periodically during teleop mode.
   */
  @Override
  public void teleopPeriodic() 
  {
    // run scheduler
    Scheduler.getInstance().run();  
  }

  /********************************************************************************************
   * Test Mode
   ********************************************************************************************/

  /**
   * This function is called 1x when the robot is 1st enabled test mode
   */
  @Override
  public void testInit() {}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  /********************************************************************************************
   * Disabled Mode
   ********************************************************************************************/
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() 
  {
    // remove all currently running commands
    Scheduler.getInstance().removeAll();

    // clear commands in all motor controllers
    _Chassis.stop(false);

    _isNotifierRunning = false;

    // optionally tell Datalogger to flush its data out to the logging destination
    if(_DataLogger != null){
      _DataLogger.close();
    }
  }

  @Override
  public void disabledPeriodic() 
  {
  }

  /********************************************************************************************
   * General Support Methods
   ********************************************************************************************/

   /**
  * This function is called every robot packet, no matter the mode. Use
  * this for items like diagnostics that you want ran during disabled,
  * autonomous, teleoperated and test.
  *
  * <p>This runs after the mode specific periodic functions, but before
  * LiveWindow and SmartDashboard integrated updating.
  */
  @Override
  public void robotPeriodic() 
  {
    // ============= Refresh Dashboard ============= 
    this.outputAllToDashboard();

    // if a notifier is running, we will call the logAllData method from inside the command so that we sync to the notifier period
    if(!this.isDisabled() && (_DataLogger != null) && !_isNotifierRunning )
    {
      this.logAllData();
    }
  }

  /** Method to Push Data to ShuffleBoard */
  private void outputAllToDashboard() 
  {
      // ----------------------------------------------
      // each subsystem should add a call to a outputToSmartDashboard method
      // to push its data out to the dashboard
      // ----------------------------------------------
      if(_Chassis != null)              { _Chassis.updateDashboard(); }
      if(_navX != null)                 { _navX.updateDashboard(); }
	}

	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
  public void logAllData() 
  { 
    // TODO: Fix ~1sec delay on DataLogger
      if(_DataLogger != null) 
      {    	
        // create a new, empty logging class
        _logData.InitData(RobotController.getFPGATime() / 1000);
        
        // ----------------------------------------------
        // ask each subsystem that exists to add its data
        // ----------------------------------------------
        if(_Chassis != null)              { _Chassis.updateLogData(_logData); }
        if(_navX != null)                 { _navX.updateLogData(_logData); }
        if((_autonomousCommand != null) 
              && (_autonomousCommand instanceof IBeakSquadDataPublisher))  
                                          { 
                                            ((IBeakSquadDataPublisher) _autonomousCommand).updateLogData(_logData); 
                                          }

	    	_DataLogger.WriteDataLine(_logData);
    	}
  }
}
