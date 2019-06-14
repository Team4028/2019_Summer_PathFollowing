/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadDataPublisher;
import frc.robot.util.MotorCtrPIDGainsBE;
import frc.robot.util.LogDataBE;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Chassis extends Subsystem implements IBeakSquadDataPublisher 
{
  private TalonSRX _leftMaster;
  private TalonSRX _leftSlave1;
  private TalonSRX _leftSlave2;
  private TalonSRX _rightMaster;
  private TalonSRX _rightSlave1;
  private TalonSRX _rightSlave2;

  private static final int CAN_TIMEOUT_MSECS_INIT = 5;
  private static final int CAN_TIMEOUT_MSECS_PERIODIC = 0;
  private static final int BIG_NUMBER = (int) 1e6;

  // Robot Physical Constants
  public static final double DRIVE_WHEEL_DIAMETER_IN = 7.0;
  private static final double TRACK_WIDTH_INCHES = 24.0;
  public static final double MAX_VEL_IN_PER_SEC = 135;
  // Encoder Constants
  private static double ENCODER_REVS_PER_WHEEL_REV = 12.0;
  private static double NU_PER_ENCODER_REV = 1024; // 256 CPR * 4 (Quad Encoder)
  public static double ENCODER_COUNTS_PER_WHEEL_REV = ENCODER_REVS_PER_WHEEL_REV * NU_PER_ENCODER_REV; // 12288;

  // PID constants
  public static final int PID_PROFILE_SLOT_IDX_HS = 0;
  public static final int PID_PROFILE_SLOT_IDX_LS = 1;
  private static final int PID_PROFILE_PID_IDX_PRIMARY = 0;

  private int _activePIDProfileSlotIdx = 0;
  private double _leftTargetVelInInchPerSec = 0;
  private double _rightTargetVelInInchPerSec = 0;

  private double _leftPercentVBusCmd;
  private double _rightPercentVBusCmd;
  /**
   * PID Gains may have to be adjusted based on the responsiveness of control
   * loop. kF: 1023 represents output value to Talon at 100%, 9200 represents
   * Velocity units at 100% output
   * 
   *                                                            kP      kI   kD       kF          Iz   PeakOut
   */

  // 100 IPS
  private final static MotorCtrPIDGainsBE PID_GAINS_LEFT_HS = new MotorCtrPIDGainsBE(0.135, 0.15, 0.0, 1023.0 / 8000.0, 300, 1.00);
  private final static MotorCtrPIDGainsBE PID_GAINS_RGT_HS = new MotorCtrPIDGainsBE(0.1225, 0.15, 0.0, 1023.0 / 8350.0, 300, 1.00);
  
  // 0-40 IPS
  private final static MotorCtrPIDGainsBE PID_GAINS_LEFT_LS = new MotorCtrPIDGainsBE(0.160, 0.25, 0.0, 1023.0 / 8000.0, 300, 1.00);
  private final static MotorCtrPIDGainsBE PID_GAINS_RGT_LS = new MotorCtrPIDGainsBE(0.1475, 0.25, 0.0, 1023.0 / 8350.0, 300, 1.00);
 

  private MotorCtrPIDGainsBE[] _pidGainsLeftLU = new MotorCtrPIDGainsBE[4];
  private MotorCtrPIDGainsBE[] _pidGainsRgtLU = new MotorCtrPIDGainsBE[4];

  // =====================================================================================
  // Define Singleton Pattern
  // =====================================================================================
  private static Chassis _instance = new Chassis();

  public static Chassis getInstance() {
    return _instance;
  }

  // private constructor for singleton pattern
  private Chassis() {
    // =============================================
    // define chassis left side motor controllers
    // =============================================
    _leftMaster = configMasterMotor(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR); // LEFT_DRIVE_MASTER_CAN_ADDR
    _leftSlave1 = configDriveMotor(RobotMap.LEFT_DRIVE_SLAVE1_CAN_ADDR);
    _leftSlave2 = configDriveMotor(RobotMap.LEFT_DRIVE_SLAVE2_CAN_ADDR);

    _leftMaster.setSensorPhase(false);

    _leftMaster.setInverted(true);
    _leftSlave1.setInverted(true);
    _leftSlave2.setInverted(true);

    _leftSlave1.follow(_leftMaster);
    _leftSlave2.follow(_leftMaster);

    // high speed
    setPIDParams(_leftMaster, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_LEFT_HS, _pidGainsLeftLU);
    setPIDParams(_leftSlave1, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_LEFT_HS, _pidGainsLeftLU);
    setPIDParams(_leftSlave2, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_LEFT_HS, _pidGainsLeftLU);

    // low speed
    setPIDParams(_leftMaster, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_LEFT_LS, _pidGainsLeftLU);
    setPIDParams(_leftSlave1, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_LEFT_LS, _pidGainsLeftLU);
    setPIDParams(_leftSlave2, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_LEFT_LS, _pidGainsLeftLU);

    // =============================================
    // define chassis right side motor controllers
    // =============================================
    _rightMaster = configMasterMotor(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);  // RIGHT_DRIVE_MASTER_CAN_ADDR
    _rightSlave1 = configDriveMotor(RobotMap.RIGHT_DRIVE_SLAVE1_CAN_ADDR);
    _rightSlave2 = configDriveMotor(RobotMap.RIGHT_DRIVE_SLAVE2_CAN_ADDR);

    _rightMaster.setSensorPhase(false);

    _rightMaster.setInverted(false);
    _rightSlave1.setInverted(false);
    _rightSlave2.setInverted(false);

    _rightSlave1.follow(_rightMaster);
    _rightSlave2.follow(_rightMaster);

    // high speed
    setPIDParams(_rightMaster, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_RGT_HS, _pidGainsRgtLU);
    setPIDParams(_rightSlave1, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_RGT_HS, _pidGainsRgtLU);
    setPIDParams(_rightSlave2, PID_PROFILE_SLOT_IDX_HS, PID_GAINS_RGT_HS, _pidGainsRgtLU);

    // low speed
    setPIDParams(_rightMaster, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_RGT_LS, _pidGainsRgtLU);
    setPIDParams(_rightSlave1, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_RGT_LS, _pidGainsRgtLU);
    setPIDParams(_rightSlave2, PID_PROFILE_SLOT_IDX_LS, PID_GAINS_RGT_LS, _pidGainsRgtLU);
  }

  private TalonSRX configMasterMotor(int canAddress) {

    TalonSRX driveMotor = configDriveMotor(canAddress);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MSECS_INIT);
    driveMotor.setSelectedSensorPosition(0, 0, CAN_TIMEOUT_MSECS_INIT);
    // talon.configGetParameter(ParamEnum.eSelectedSensorCoefficient, ordinal);

    // talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
    // CAN_TIMEOUT_MSECS_INIT);
    // talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,
    // CAN_TIMEOUT_MSECS_INIT);
    // talon.configVelocityMeasurementWindow(32, CAN_TIMEOUT_MSECS_INIT);

    driveMotor.configClosedloopRamp(0.0, CAN_TIMEOUT_MSECS_INIT);

    // update PID 0 feedback values more often (default = 100mS) so error value is more accurate
    driveMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, CAN_TIMEOUT_MSECS_INIT);
    return driveMotor;
  }

  private TalonSRX configDriveMotor(int canAddress) {
    TalonSRX driveMotor = new TalonSRX(canAddress);
    driveMotor.configFactoryDefault();
    driveMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,
        CAN_TIMEOUT_MSECS_INIT);
    driveMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,
        CAN_TIMEOUT_MSECS_INIT);

    driveMotor.enableCurrentLimit(false);
    driveMotor.configOpenloopRamp(0, CAN_TIMEOUT_MSECS_INIT);

    driveMotor.configPeakOutputForward(1.0, CAN_TIMEOUT_MSECS_INIT);
    driveMotor.configPeakOutputReverse(-1.0, CAN_TIMEOUT_MSECS_INIT);
    driveMotor.configNominalOutputForward(0, CAN_TIMEOUT_MSECS_INIT);
    driveMotor.configNominalOutputReverse(0, CAN_TIMEOUT_MSECS_INIT);
    driveMotor.configContinuousCurrentLimit(BIG_NUMBER, CAN_TIMEOUT_MSECS_INIT);

    driveMotor.setNeutralMode(NeutralMode.Brake);

    return driveMotor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // =====================================================================================
  // Public Helper Methods
  // =====================================================================================
  // basic driving
  public void arcadeDrive(double rawThrottleCmd, double rawTurnCmd) {

    double leftAdj = 1.0;
    double rightAdj = 0.95;   // robot is verring to left (overdriving on right side)

    double lowEndPowerFactor = 1.0;
    double sign = (rawThrottleCmd < 0) ? -1.0 : 1.0;

    // apply some non-linear scaling to the low ranges
    double adjLeftThrottleCmd = Math.pow(Math.abs(rawThrottleCmd) * leftAdj, lowEndPowerFactor) * sign;
    double adjRgtThrottleCmd = Math.pow(Math.abs(rawThrottleCmd) * rightAdj, lowEndPowerFactor) * sign;

    // really scale down turn commands
    double adjLeftTurnCmd = rawTurnCmd * 0.3;
    double adjRgtTurnCmd = rawTurnCmd * 0.3;

    _leftMaster.set(ControlMode.PercentOutput, adjLeftThrottleCmd + adjLeftTurnCmd);
    _rightMaster.set(ControlMode.PercentOutput, adjRgtThrottleCmd - adjRgtTurnCmd);
  }

  public void zeroSensors()
  {
    _leftMaster.getSensorCollection().setQuadraturePosition(0, CAN_TIMEOUT_MSECS_PERIODIC);
    _rightMaster.getSensorCollection().setQuadraturePosition(0, CAN_TIMEOUT_MSECS_PERIODIC);
  }

  public void stop(boolean isSetBrakeMode) 
  {
    setOpenLoopVelocityCmd(0.0, 0.0);
    if(isSetBrakeMode)
    {
      setBrakeMode(NeutralMode.Brake);
    }
    else
    {
      setBrakeMode(NeutralMode.Coast);
    }
  }

  public void setBrakeMode(NeutralMode mode)
  {
    if (_leftMaster != null)  _leftMaster.setNeutralMode(mode);
    if (_leftSlave1 != null)  _leftSlave1.setNeutralMode(mode);
    if (_leftSlave2 != null)  _leftSlave2.setNeutralMode(mode);

		if (_rightMaster != null)  _rightMaster.setNeutralMode(mode);
    if (_rightSlave1 != null)  _rightSlave1.setNeutralMode(mode);
    if (_rightSlave2 != null)  _rightSlave2.setNeutralMode(mode);
  }

  public void setActivePIDConstantsSlot(int slotIdx)
  {
    _leftMaster.selectProfileSlot(slotIdx, PID_PROFILE_PID_IDX_PRIMARY);
    _rightMaster.selectProfileSlot(slotIdx, PID_PROFILE_PID_IDX_PRIMARY);

    _activePIDProfileSlotIdx = slotIdx;
  }

  public void setOpenLoopVelocityCmd(double leftPercentVBusCmd, double rightPercentVBusCmd)
  {
    _leftPercentVBusCmd = leftPercentVBusCmd;
    _rightPercentVBusCmd = rightPercentVBusCmd;

    _leftMaster.set(ControlMode.PercentOutput, leftPercentVBusCmd);
    _rightMaster.set(ControlMode.PercentOutput, rightPercentVBusCmd);
  }

  public void setClosedLoopVelocityCmd(double leftTargetVelInInchPerSec, double rightTargetVelInInchPerSec)
  {
    _leftTargetVelInInchPerSec = leftTargetVelInInchPerSec;
    _rightTargetVelInInchPerSec = rightTargetVelInInchPerSec;

    _leftMaster.set(ControlMode.Velocity, convertInchesPerSecToNUPer100mS(leftTargetVelInInchPerSec));
    _rightMaster.set(ControlMode.Velocity, convertInchesPerSecToNUPer100mS(rightTargetVelInInchPerSec));
  }

  // =====================================================================================
  // Private Helper Methods
  // =====================================================================================

  private double CalcNUPer100mSFromRPM(double wheelRPM) {

    double kMinPerSec = 1.0 / 60.0;
    double kSecPer100mSec = 1.0 / 10.0;

    double rpmEncoder = wheelRPM * ENCODER_REVS_PER_WHEEL_REV;

    double encoderRevPer100mS = rpmEncoder * kMinPerSec * kSecPer100mSec;

    double nuPer100mS = encoderRevPer100mS * NU_PER_ENCODER_REV;

    return nuPer100mS;
  }

  private void setPIDParams(TalonSRX talon, int pidSlotIDX, MotorCtrPIDGainsBE pidGains, MotorCtrPIDGainsBE[] pidGainsLU) {
    // Config the Velocity closed loop gains
    talon.config_kF(pidSlotIDX, pidGains.KF, CAN_TIMEOUT_MSECS_INIT);
    talon.config_kP(pidSlotIDX, pidGains.KP, CAN_TIMEOUT_MSECS_INIT);
    talon.config_kI(pidSlotIDX, pidGains.KI, CAN_TIMEOUT_MSECS_INIT);
    talon.config_kD(pidSlotIDX, pidGains.KD, CAN_TIMEOUT_MSECS_INIT);

    talon.configMaxIntegralAccumulator(pidSlotIDX, pidGains.KMaxI, CAN_TIMEOUT_MSECS_INIT);

    pidGainsLU[pidSlotIDX] = pidGains;
  }

  // Postion
  public double getLeftEncoderPositionInNU() {
    return _leftMaster.getSelectedSensorPosition(0);
  }

  public double getLeftMotorOutputVoltage()
  {
    return _leftMaster.getMotorOutputVoltage();
  }

  public double getRightMotorOutputVoltage()
  {
    return _rightMaster.getMotorOutputVoltage();
  }

  public double getRightEncoderPositionInNU() {
    return _rightMaster.getSelectedSensorPosition(0);
  }

  public double getLeftChassisPositionInInches() {
    return (getLeftEncoderPositionInNU() / ENCODER_COUNTS_PER_WHEEL_REV) * DRIVE_WHEEL_DIAMETER_IN * Math.PI;
  }

  public double getRightChassisPositionInInches() {
    return (getRightEncoderPositionInNU() / ENCODER_COUNTS_PER_WHEEL_REV) * DRIVE_WHEEL_DIAMETER_IN * Math.PI;
  }

  // Velocity
  public double getLeftEncoderVelocityInNUPer100mS() {
    return _leftMaster.getSelectedSensorVelocity(0);
  }

  public double getRightEncoderVelocityInNUPer100mS() {
    return _rightMaster.getSelectedSensorVelocity(0);
  }

  public double getLeftEncoderClosedLoopTarget() 
  {
    if (_leftMaster.getControlMode() != ControlMode.PercentOutput)
    {
      return _leftMaster.getClosedLoopTarget(0);
    }
    else
    {
      return 0;
    }
  }

  public double getRightEncoderClosedLoopTarget()
  {
    if (_rightMaster.getControlMode() != ControlMode.PercentOutput)
    {
      return _rightMaster.getClosedLoopTarget(0);
    }
    else
    {
      return 0;
    }
  }

  public double getLeftEncoderClosedLoopError()
  {
    if (_leftMaster.getControlMode() != ControlMode.PercentOutput)
    {
      return _leftMaster.getClosedLoopError(0);
    }
    else
    {
      return 0;
    }
  }

  public double getRightEncoderClosedLoopError() 
  {
    if (_rightMaster.getControlMode() != ControlMode.PercentOutput)
    {
      return _rightMaster.getClosedLoopError(0);
    }
    else
    {
      return 0;
    }
  }

  public double getLeftWheelVelocityInRPM() {
    return getLeftEncoderVelocityInNUPer100mS() * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
  }

  public double getRightWheelVelocityInRPM() {
    return getRightEncoderVelocityInNUPer100mS() * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
  }

  private double getLeftChassisVelocityInInchesPerSec() {
    return convertRPMToInchesPerSecond(getLeftWheelVelocityInRPM());
  }

  private double getRightChassisVelocityInInchesPerSec() {
    return convertRPMToInchesPerSecond(getRightWheelVelocityInRPM());
  }

  // =====================================================================================
  // Private Conversion Helper Methods
  // =====================================================================================

  private static double convertRPMToInchesPerSecond(double rpm) {
    return convertRotationsToInches(rpm) / 60;
  }

  private static double convertRotationsToInches(double rot) {
    return rot * (DRIVE_WHEEL_DIAMETER_IN * Math.PI);
  }

  private static double convertInchestoNU(double inches) {
    return (inches * ENCODER_COUNTS_PER_WHEEL_REV) / (DRIVE_WHEEL_DIAMETER_IN * Math.PI);
  }

  private static double convertNUtoInches(double nu) {
    return (nu * DRIVE_WHEEL_DIAMETER_IN * Math.PI) / ENCODER_COUNTS_PER_WHEEL_REV;
  }

  private static double convertInchesPerSecToNUPer100mS(double inchesPerSecond) {
    return (inchesPerSecond * ENCODER_COUNTS_PER_WHEEL_REV) / (DRIVE_WHEEL_DIAMETER_IN * Math.PI * 10);
  }

  private static double convertInchesPerSecToPercentOut(double inchesPerSecond) {
    return (12.0 / Chassis.MAX_VEL_IN_PER_SEC) * inchesPerSecond;
  }

  private static double convertNUper100msToInchesPerSec(double nuPer100ms) {
    return (nuPer100ms * 10 * DRIVE_WHEEL_DIAMETER_IN * Math.PI) / ENCODER_COUNTS_PER_WHEEL_REV;
  }

  @Override
  public void updateLogData(LogDataBE logData) 
  {
    // Left Log Values
    logData.AddData("Chassis:LeftPercentVBusCmd", Double.toString(_leftPercentVBusCmd));
    logData.AddData("Chassis:LeftEncClosedLoopIPS", Double.toString(_leftTargetVelInInchPerSec));
    logData.AddData("Chassis:LeftEncClosedLoopTrgt", Double.toString(getLeftEncoderClosedLoopTarget()));
    logData.AddData("Chassis:LeftEncPosInNU", Double.toString(getLeftEncoderPositionInNU()));
    logData.AddData("Chassis:LeftEncActualVelInNUPer100mS", Double.toString(getLeftEncoderVelocityInNUPer100mS()));
    logData.AddData("Chassis:LeftEncClosedLoopErr", Double.toString(getLeftEncoderClosedLoopError()));

    logData.AddData("Chassis:LeftChasPosInInches", Double.toString(getLeftChassisPositionInInches()));
    logData.AddData("Chassis:LeftChasVelInIPS", Double.toString(getLeftChassisVelocityInInchesPerSec()));
    logData.AddData("Chassis:LeftControlMode", _leftMaster.getControlMode().toString());
    logData.AddData("Chassis:LeftOutputPercent", Double.toString(_leftMaster.getMotorOutputPercent()));
    MotorCtrPIDGainsBE leftCurrentGains = _pidGainsLeftLU[_activePIDProfileSlotIdx];
    String leftGains = Double.toString(leftCurrentGains.KF) + " | " + 
                        Double.toString(leftCurrentGains.KP) + " | " + 
                        Double.toString(leftCurrentGains.KI) + " | " + 
                        Double.toString(leftCurrentGains.KD);
    logData.AddData("Chassis:LeftPIDGains", leftGains);
    
    // Right Log Values
    logData.AddData("Chassis:RgtPercentVBusCmd", Double.toString(_rightPercentVBusCmd));
    logData.AddData("Chassis:RgtEncClosedLoopIPS", Double.toString(_rightTargetVelInInchPerSec));
    logData.AddData("Chassis:RgtEncClosedLoopTrgt", Double.toString(getRightEncoderClosedLoopTarget()));
    logData.AddData("Chassis:RgtEncPosInNU", Double.toString(getRightEncoderPositionInNU()));
    logData.AddData("Chassis:RgtEncActualVelInNUPer100mS", Double.toString(getRightEncoderVelocityInNUPer100mS()));
    logData.AddData("Chassis:RgtEncClosedLoopErr", Double.toString(getRightEncoderClosedLoopError()));

    logData.AddData("Chassis:RgtChasPosInInches", Double.toString(getRightChassisPositionInInches()));
    logData.AddData("Chassis:RgtChasVelInIPS", Double.toString(getRightChassisVelocityInInchesPerSec()));
    logData.AddData("Chassis:RgtControlMode", _rightMaster.getControlMode().toString());
    logData.AddData("Chassis:RgtOutputPercent", Double.toString(_rightMaster.getMotorOutputPercent()));
    MotorCtrPIDGainsBE rightCurrentGains = _pidGainsRgtLU[_activePIDProfileSlotIdx];
    String rightGains = Double.toString(rightCurrentGains.KF) + " | " + 
                        Double.toString(rightCurrentGains.KP) + " | " + 
                        Double.toString(rightCurrentGains.KI) + " | " + 
                        Double.toString(rightCurrentGains.KD);
    logData.AddData("Chassis:RgtPIDGains", rightGains);
  }

  @Override
  public void updateDashboard() 
  {
    SmartDashboard.putNumber("Chassis:LeftEncPosInNU", getLeftEncoderPositionInNU());
    SmartDashboard.putNumber("Chassis:LeftChasPosInInches", getLeftChassisPositionInInches());
    SmartDashboard.putNumber("Chassis:LeftEncClosedLoopTrgt", getLeftEncoderClosedLoopTarget());
    SmartDashboard.putNumber("Chassis:LeftEncClosedLoopErr", getLeftEncoderClosedLoopError());
    SmartDashboard.putNumber("Chassis:LeftMtrOutputVolts", getLeftMotorOutputVoltage());
    SmartDashboard.putNumber("Chassis:RgtEncPosInNU", getRightEncoderPositionInNU());
    SmartDashboard.putNumber("Chassis:RgtChasPosInInches", getRightChassisPositionInInches());
    SmartDashboard.putNumber("Chassis:RgtEncClosedLoopTrgt", getRightEncoderClosedLoopTarget());
    SmartDashboard.putNumber("Chassis:RgtEncClosedLoopErr", getRightEncoderClosedLoopError());
    SmartDashboard.putNumber("Chassis:RgtMtrOutputVolts", getRightMotorOutputVoltage());
  }

}
