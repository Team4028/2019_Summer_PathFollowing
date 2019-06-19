package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.chassis.DriveClosedLoopVelocity;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.chassis.ZeroSensors;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController;
import frc.robot.util.DataLogger;

/**
 * This class interfaces with the Driver/Operator Station Lead Student:
 */
public class OI {
	private BeakXboxController _driverController;
	private BeakXboxController _operatorController;
	private BeakXboxController _engineerController;

	private DataLogger _dataLoggerInstance;

	// =====================================================================================
	// Define Singleton Pattern
	// =====================================================================================
	private static OI _instance = new OI();

	public static OI getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private OI() 	
	{	
		// =========== Driver ======================================
		_driverController = new BeakXboxController(RobotMap.DRIVER_GAMEPAD_USB_PORT);
		//==========================================================

		// Driver Controller -> Command Mapping
		//_driverController.leftStick.whileActive(new DriveWithControllers());	
		//_driverController.leftStick.whenReleased(new DriveWithControllers());	// this stops the 
		//_driverController.rightStick.whileActive(new DriveWithControllers());	
		//_driverController.rightStick.whenReleased(new DriveWithControllers());

		//															run time = 10 sec
		//															target vel = 12 in/sec
		_driverController.a.whenPressed(new DriveClosedLoopVelocity("3_40", Chassis.PID_PROFILE_SLOT_IDX_LS, 3, 40.0, 40.0));
		_driverController.b.whenPressed(new StopChassis());
		_driverController.x.whenPressed(new ZeroSensors());

		// =========== Operator ======================================
		//_operatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		// ==========================================================


		// =========== Engineer ======================================
		// _engineerController = new BeakXboxController(RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
		// //============================================================

	}

	/**
 	* @return the raw controller throttle
	*/
	public double getThrottleCmd() 
	{
		// -1.0 (up / forward)
		//  ^
		//  |
		//  v 
		// 1.0 (down / reverse)
		return (_driverController.leftStick.getY() * 1.0);
	}

	/**
 	* @return the raw turn throttle
	*/
	public double getTurnCmd() 
	{
		return (_driverController.rightStick.getX() * 1.0);
	}
}

