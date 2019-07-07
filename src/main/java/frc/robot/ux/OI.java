package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.chassis.DriveClosedLoopFixedVelocity;
import frc.robot.commands.chassis.DriveClosedLoopStepVelocity;
import frc.robot.commands.chassis.DriveOpenLoopStepVBus;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.chassis.ZeroSensors;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController;

/**
 * This class interfaces with the Driver/Operator Station 
 * 
 * Lead Student:
 */
public class OI {
	private BeakXboxController _driverController;
	// private BeakXboxController _operatorController;
	// private BeakXboxController _engineerController;

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
		//															run time = 10 sec
		//															target vel = 12 in/sec
		_driverController.a.whenPressed(new DriveClosedLoopFixedVelocity("3_20", Chassis.PID_PROFILE_SLOT_IDX_LS, 3, 20.0, 20.0));
		_driverController.b.whenPressed(new StopChassis());
		_driverController.x.whenPressed(new ZeroSensors());
		_driverController.y.whenPressed(new DriveOpenLoopStepVBus(25, 0.025));
		//_driverController.y.whenPressed(new DriveClosedLoopStepVelocity(30, 0.25, Chassis.PID_PROFILE_SLOT_IDX_LS));

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

