/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.chassis.DriveFollowPathOpenLoopV3;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.interfaces.IDataLogger;
import frc.robot.interfaces.LogDestination;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.DataLogger;
import frc.robot.util.DataLoggerV2;
import frc.robot.util.GeneralUtilities;
import frc.robot.ux.OI;
import frc.robot.ux.PathChooser;
import frc.robot.entities.LogDataBE;

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

  // sensors
  public static GyroNavX _NavX = GyroNavX.getInstance();

  // subsystems
  public static Chassis _Chassis = Chassis.getInstance();

  // ux
  public static OI _OI = OI.getInstance();
  private static PathChooser _pathChooser = PathChooser.getInstance();

  // class level working variables
  // flushing at -1 means only bufer and write when disabled
  public static IDataLogger _DataLogger = new DataLoggerV2(LogDestination.USB, -1);
  private Command _autonomousCommand = null;
  //private DriveFollowPathOpenLoop _autonomousCommand = null;
  private boolean _isNotifierRunning = false;
  private LogDataBE _logData;

  private static final boolean IS_VERBOSE_LOGGING_ENABLED = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // https://www.chiefdelphi.com/t/improbable-java-slow-down-in-combining-doubles-and-strings/350331/17
    System.out.println("WarmUP Complete?: " + GeneralUtilities.WarmUp());

    // write the overall robot dashboard info
    SmartDashboard.putString("Robot Build", GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME));

    _NavX.zeroYaw();
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
    _NavX.zeroYaw();

    // init data logging
    _DataLogger.initLogging("Auton"); 
    _isNotifierRunning = true;

    // setup auton command
    //_autonomousCommand = new DriveFollowPathOpenLoopV3("Straight240in", this::logAllData);
    _autonomousCommand = new DriveFollowPathOpenLoopV3("LeftTurn6ftR", this::logAllData);
    //_autonomousCommand = new DriveFollowPathOpenLoopV2("Straight240in", this::logAllData);
    // schedule the autonomous command
    if (_autonomousCommand != null) {
      _autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // run the command scheduler
    Scheduler.getInstance().run();
  }

  /********************************************************************************************
   * Telop Mode
   ********************************************************************************************/

  @Override
  public void teleopInit() {   
    // zero sensors at start of telop
    _Chassis.zeroSensors();
    _NavX.zeroYaw();

    // init data logging
    _DataLogger.initLogging("Telop");
    _isNotifierRunning = false;

    // start honoring joysticks
    Command driveWJoyStick = new DriveWithControllers();
    driveWJoyStick.start();
  }

   /* This function is called periodically during teleop mode.
   */
  @Override
  public void teleopPeriodic() {
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
  public void disabledInit() {
    // remove all currently running commands
    Scheduler.getInstance().removeAll();
    // reset auton
    _autonomousCommand = null;

    // clear commands in all motor controllers
    _Chassis.stop(false);

    _isNotifierRunning = false;

    // optionally tell Datalogger to flush its data out to the logging destination
    if(_DataLogger != null){
      _DataLogger.close();
    }
  }

  @Override
  public void disabledPeriodic() {}

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
  public void robotPeriodic() {
    // ============= Refresh Dashboard ============= 
    this.outputAllToDashboard();

    // if a notifier is running, we will call the logAllData method from inside the command so that we sync to the notifier period
    if(!this.isDisabled() 
          && (_DataLogger != null) 
          && !_isNotifierRunning 
          && _autonomousCommand == null) {
      this.logAllData();
    }
  }

  /** Method to Push Data to ShuffleBoard */
  private void outputAllToDashboard() {
      // ----------------------------------------------
      // each subsystem should add a call to a outputToSmartDashboard method
      // to push its data out to the dashboard
      // ----------------------------------------------
      if(_Chassis != null)              { _Chassis.updateDashboard(); }
      if(_NavX != null)                 { _NavX.updateDashboard(); }
      if((_autonomousCommand != null) 
              && (_autonomousCommand instanceof IBeakSquadDataPublisher))  
                                          { 
                                            ((IBeakSquadDataPublisher) _autonomousCommand).updateDashboard(); 
                                          }
      if(_pathChooser != null)          { _pathChooser.updateDashboard(); }
	}

	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
  public void logAllData() { 

      // create a new, empty logging class
      _logData = new LogDataBE(GeneralUtilities.getRoundedFPGATime());
 
      // ----------------------------------------------
      // ask each subsystem that exists to add its data
      // ----------------------------------------------
      if(_Chassis != null)              { _Chassis.updateLogData(_logData, IS_VERBOSE_LOGGING_ENABLED); }
      _logData.AddData("logAllData:AfterChassis", Long.toString(RobotController.getFPGATime() / 1000));

      if(_NavX != null)                 { _NavX.updateLogData(_logData, IS_VERBOSE_LOGGING_ENABLED); }
      _logData.AddData("logAllData:AfterNavX", Long.toString(RobotController.getFPGATime() / 1000));

      // if the auton command is set 
      if((_autonomousCommand != null)
           && (_autonomousCommand instanceof IBeakSquadDataPublisher))  
                                        { ((IBeakSquadDataPublisher) _autonomousCommand).updateLogData(_logData, IS_VERBOSE_LOGGING_ENABLED); }
      _logData.AddData("logAllData:AfterAuton", Long.toString(RobotController.getFPGATime() / 1000));

      //if(_pathChooser != null)          { _pathChooser.updateLogData(_logData, IS_VERBOSE_LOGGING_ENABLED); }    
      _logData.AddData("logAllData:AtEnd", Long.toString(RobotController.getFPGATime() / 1000));

      _DataLogger.LogData(_logData);
  }
}
