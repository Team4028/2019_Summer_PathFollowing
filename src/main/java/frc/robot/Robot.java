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

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.DataLogger;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.LogDataBE;
import frc.robot.ux.OI;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

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
  //  Note: add each one to the outputAllToDashboard & logAllData methods below
  // ux
  private OI _oi = OI.getInstance();

  // sensors
  private GyroNavX _navX = GyroNavX.getInstance();

  // subsystems
  public static Chassis _Chassis = Chassis.getInstance();

  // class level working variables
  public static DataLogger _DataLogger = null;
  private String _buildMsg = "?";

  //=====================================================

  // number of encoder counts per wheel revolution
  private static final int k_ticks_per_rev = 1024;
  // diameter of the wheels
  private static final double k_wheel_diameter = 4.0 / 12.0;
  // maximum velocity of the robot
  private static final double k_max_velocity = 10;

  // the port numbers for the left and right speed controllers
  private static final int k_left_channel = 0;
  // the port numbers for the left and right speed controllers
  private static final int k_right_channel = 1;

  // the port numbers for the encoders connected to the left and right side of the drivetrain
  private static final int k_left_encoder_port_a = 0;
  private static final int k_left_encoder_port_b = 1;
  private static final int k_right_encoder_port_a = 2;
  private static final int k_right_encoder_port_b = 3;

  // the analog input for the gyro (other gyros might be connected differently)
  private static final int k_gyro_port = 0;

  // name of this path
  private static final String k_path_name = "example";

  private SpeedController _left_motor;
  private SpeedController _right_motor;

  private Encoder _left_encoder;
  private Encoder _right_encoder;

  //private AnalogGyro _gyro;

  private EncoderFollower _left_follower;
  private EncoderFollower _right_follower;
  
  private Notifier _follower_notifier;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    _buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);

    //_navX = GyroNavX.getInstance();
    //_chassis = Chassis.getInstance();
    
    //_oi = OI.getInstance();

   // _left_motor = new Spark(k_left_channel);
    //_right_motor = new Spark(k_right_channel);

    //_left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
    //_right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);

    //_gyro = new AnalogGyro(k_gyro_port);
  }

  /********************************************************************************************
   * Autonomous Mode
   ********************************************************************************************/
  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() 
  {
    _Chassis.zeroSensors();
    _Chassis.stop(true);
    _Chassis.setBrakeMode(NeutralMode.Brake);

    _navX.zeroYaw();

    _DataLogger = GeneralUtilities.setupLogging("Auton"); // init data logging	

    /*
    Trajectory left_trajectory = null; //PathfinderFRC.getTrajectory(k_path_name + ".left");
    Trajectory right_trajectory = null; //PathfinderFRC.getTrajectory(k_path_name + ".right");

    _left_follower = new EncoderFollower(left_trajectory);
    _right_follower = new EncoderFollower(right_trajectory);

    _left_follower.configureEncoder(_left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    _left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    _right_follower.configureEncoder(_right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    _right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    
    _follower_notifier = new Notifier(this::followPath);
    _follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    */
  }

  private void followPath() 
  {
    if (_left_follower.isFinished() || _right_follower.isFinished()) 
    {
      _follower_notifier.stop();
    } 
    else 
    {
      double left_speed = _left_follower.calculate(_left_encoder.get());
      double right_speed = _right_follower.calculate(_right_encoder.get());

      // If you have a typical gyro, then it gives a + reading for a clockwise rotation \
      // where Pathfinder expects this to be a negative gyro direction.
      double heading = 0;// _gyro.getAngle();
      double desired_heading = Pathfinder.r2d(_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;

      _left_motor.set(left_speed + turn);
      _right_motor.set(right_speed - turn);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /********************************************************************************************
   * Telop Mode
   ********************************************************************************************/

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //_follower_notifier.stop();
    //_left_motor.set(0);
    //_right_motor.set(0);

    _Chassis.stop(true);
    _navX.zeroYaw();
    
    _DataLogger = GeneralUtilities.setupLogging("Telop"); // init data logging	
  }

   /* This function is called periodically during teleop mode.
   */
  @Override
  public void teleopPeriodic() 
  {
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
    Scheduler.getInstance().removeAll();

    // clear commands in all motor controllers
    _Chassis.stop(false);
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

    if(!isDisabled()) 
    {
      // ============= Optionally Log Data =============
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

      // write the overall robot dashboard info
      SmartDashboard.putString("Robot Build", _buildMsg);
	}

	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
  private void logAllData() 
  { 
		// always call this 1st to calc drive metrics
      if(_DataLogger != null) 
      {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
        
        // ----------------------------------------------
        // ask each subsystem that exists to add its data
        // ----------------------------------------------
        if(_Chassis != null)              { _Chassis.updateLogData(logData); }
        if(_navX != null)                 { _navX.updateLogData(logData); }
	    	_DataLogger.WriteDataLine(logData);
    	}
	}
}
