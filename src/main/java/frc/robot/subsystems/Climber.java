// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;

public class Climber extends SubsystemBase implements Loggable {
  private int logRotationKey;           // Key to identify the log rotation
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follows normal logging cycles
  private String subsystemName;         // Used in file logging and SmartDashboard for naming log entries

  // Motors
  private final TalonFX climberLeftMotor = new TalonFX(Ports.CANClimberMotorLeft);
  private final TalonFX climberRightMotor = new TalonFX(Ports.CANClimberMotorRight);

  // Sensors
  private final DigitalInput lowerLimitLeft = new DigitalInput(Ports.DIOClimberLowerLimitLeftSensor);
  private final DigitalInput lowerLimitRight = new DigitalInput(Ports.DIOClimberLowerLimitRightSensor);
  private final DigitalInput upperLimitLeft = new DigitalInput(Ports.DIOClimberUpperLimitLeftSensor);
  private final DigitalInput upperLimitRight = new DigitalInput(Ports.DIOClimberUpperLimitRightSensor);

  // Configurators
  private final TalonFXConfigurator climberLeftConfigurator = climberLeftMotor.getConfigurator();
  private final TalonFXConfigurator climberRightConfigurator = climberRightMotor.getConfigurator();

  // Configurations
  private TalonFXConfiguration climberLeftConfig;
  private TalonFXConfiguration climberRightConfig;

  // Status signals for the left climber motor
  private final StatusSignal<Temperature> climberLeftTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> climberLeftStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> climberLeftVoltage;                  // Motor voltage, in volts

  // Status signals for the left climber motor
  private final StatusSignal<Temperature> climberRightTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> climberRightStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> climberRightVoltage;                  // Motor voltage, in volts

  private VoltageOut climberVoltageControl = new VoltageOut(0.0).withEnableFOC(false);

    // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogClimberLeftTemp = new DoubleLogEntry(log, "/ClimberLeft/Temperature");
  private final DoubleLogEntry dLogClimberLeftCurrent = new DoubleLogEntry(log, "/ClimberLeft/Current");
  private final DoubleLogEntry dLogClimberLeftVoltage = new DoubleLogEntry(log, "/ClimberLeft/Voltage");
  private final DoubleLogEntry dLogClimberRightTemp = new DoubleLogEntry(log, "/ClimberRight/Temperature");
  private final DoubleLogEntry dLogClimberRightCurrent = new DoubleLogEntry(log, "/ClimberRight/Current");
  private final DoubleLogEntry dLogClimberRightVoltage = new DoubleLogEntry(log, "/ClimberRight/Voltage");

  // Current setpoint percent output (logical percent; sign indicates direction)
  private double setpointPercentLeft;
  private double setpointPercentRight;

  // Sticky Faults
  private boolean climberLeftMissingStickyFault;
  private boolean climberRightMissingStickyFault;
  private boolean climberLeftLowerLimitMissingStickyFault;
  private boolean climberRightLowerLimitMissingStickyFault;
  private boolean climberLeftUpperLimitMissingStickyFault;
  private boolean climberRightUpperLimitMissingStickyFault;

  public Climber(String subsystemName) {
    this.subsystemName = subsystemName;
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Get status signals for left climber motor
    climberLeftTemp = climberLeftMotor.getDeviceTemp();
    climberLeftStatorCurrent = climberLeftMotor.getStatorCurrent();
    climberLeftVoltage = climberLeftMotor.getMotorVoltage();

    // Get status signals for left climber motor
    climberRightTemp = climberRightMotor.getDeviceTemp();
    climberRightStatorCurrent = climberRightMotor.getStatorCurrent();
    climberRightVoltage = climberRightMotor.getMotorVoltage();

    // Configure motors independently (they drive opposite physical directions).
    // Logical percent sign is consistent across both sides.
    climberLeftConfig = new TalonFXConfiguration();
    climberLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberLeftConfig.Voltage.PeakForwardVoltage = UptakeConstants.compensationVoltage;
    climberLeftConfig.Voltage.PeakReverseVoltage = -UptakeConstants.compensationVoltage;
    climberLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    climberLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds
    climberLeftConfig.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
    climberLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberRightConfig = new TalonFXConfiguration();
    // Requirement: right side should be counterclockwise_positive.
    climberRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberRightConfig.Voltage.PeakForwardVoltage = UptakeConstants.compensationVoltage;
    climberRightConfig.Voltage.PeakReverseVoltage = -UptakeConstants.compensationVoltage;
    climberRightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
    climberRightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;
    climberRightConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    climberRightConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
    climberRightConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    climberRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to the motors.
    // This is a blocking call and will wait up to 200ms to apply.
    climberLeftConfigurator.apply(climberLeftConfig);
    Wait.waitTime(250);
    climberRightConfigurator.apply(climberRightConfig);
    Wait.waitTime(250);

    // Stop the climber motors
    setpointPercentLeft = 0.0;
    setpointPercentRight = 0.0;
    stopClimberMotors();

    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);

  }

  /**
   * Sets the percent output of both climber motors using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
   */
  public void setClimberPercentOutput(double percent) {
    setClimberPercentOutput(percent, ClimberConstants.SIDE_LEFT);
    setClimberPercentOutput(percent, ClimberConstants.SIDE_RIGHT);
  }

  /**
   * Sets the percent output of one climber motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
   * @param side which climber side to control
   */
  public void setClimberPercentOutput(double percent, int side) {
    if (side == ClimberConstants.SIDE_LEFT) {
      setpointPercentLeft = percent;
      climberLeftMotor.setControl(climberVoltageControl.withOutput(percent * ClimberConstants.compensationVoltage));
    } else {
      setpointPercentRight = percent;
      climberRightMotor.setControl(climberVoltageControl.withOutput(percent * ClimberConstants.compensationVoltage * 0.995));
    }
  }

  /**
   * Stops the Climber motor.
   */
  public void stopClimberMotors() {
    setClimberPercentOutput(0);
  }

  /**
   * Stops a single climber motor.
   * @param side which climber side to control
   */
  public void stopClimberMotor(int side) {
    setClimberPercentOutput(0.0, side);
  }

  /**
   * Gets the setpoint percent output to the climber motors.
   * @return left side percent output (legacy; prefer side-specific getters)
   */
  public double getClimberSetpointPercent() {
    return setpointPercentLeft;
  }

  /** @return left side percent output (logical percent) */
  public double getClimberSetpointPercentLeft() {
    return getClimberSetpointPercent(ClimberConstants.SIDE_LEFT);
  }

  /** @return right side percent output (logical percent) */
  public double getClimberSetpointPercentRight() {
    return getClimberSetpointPercent(ClimberConstants.SIDE_RIGHT);
  }

  /**
   * Returns the current side setpoint percent output (logical percent).
   *
   * Side convention:
   * 1 = left, 2 = right
   *
   * @param side which side to query
   * @return logical percent output being applied to that side
   */
  public double getClimberSetpointPercent(int side) {
    return side == ClimberConstants.SIDE_LEFT ? setpointPercentLeft : setpointPercentRight;
  }

  /**
   * Gets the temperature of the Climber motor
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return climber temperature, in degrees celsius
   */
  public double getClimberTemp(int motorNumber) {
    return motorNumber == 1 ? climberLeftTemp.refresh().getValueAsDouble() : climberRightTemp.refresh().getValueAsDouble();
  }

  /**
   * Gets the stator current of the Climber motor.
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return motor current, in amps
   */
  public double getClimberAmps(int motorNumber) {
    return motorNumber == 1 ? climberLeftStatorCurrent.refresh().getValueAsDouble() : climberRightStatorCurrent.refresh().getValueAsDouble();
  }

  /**
   * Reads the voltage applied to the Climber motor.
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return climber voltage, in volts
   */
  public double getClimberVoltage(int motorNumber) {
    return motorNumber == 1 ? climberLeftVoltage.refresh().getValueAsDouble() : climberRightVoltage.refresh().getValueAsDouble();
  }

  /**
   * Returns if the climber is in trench configuration
   * Sometimes only one limit triggers when all the way down, 
   * so we just check if either limit is triggered
   * @return true = climber is all the way down, false = climber isn't all the way down.
   */
  public boolean isSafeForTrench() {
    return !lowerLimitLeft.get() || !lowerLimitRight.get();
  }

  /**
   * Checks if the provided side is physically at a limit.
   *
   * @param side 1 = left, 2 = right
   * @param lower true = lower limit, false = upper limit
   * @return true if the side is physically at the selected limit
   */
  public boolean isSideLimitReached(int side, boolean lower) {
    if (side == ClimberConstants.SIDE_LEFT) return lower ? !lowerLimitLeft.get() : !upperLimitLeft.get();
    return lower ? !lowerLimitRight.get() : !upperLimitRight.get();
  }

  /**
   * Checks whether the provided setpoint would be blocked by the given side's limits.
   *
   * Convention:
   * - setpointPercent < 0.0 means moving toward the lower limit
   * - setpointPercent > 0.0 means moving toward the upper limit
   */
  public boolean isSideLimitHitForSetpoint(int side, double setpointPercent) {
    if (setpointPercent < 0.0) return isSideLimitReached(side, true);
    if (setpointPercent > 0.0) return isSideLimitReached(side, false);
    return false;
  }

  /**
   * Checks whether either side would be blocked by the provided setpoint direction.
   */
  public boolean isEitherSideLimitHitForSetpoint(double setpointPercent) {
    return isSideLimitHitForSetpoint(ClimberConstants.SIDE_LEFT, setpointPercent)
        || isSideLimitHitForSetpoint(ClimberConstants.SIDE_RIGHT, setpointPercent);
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Turns file logging on every scheduler cycle (~20 msec) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about the Climber to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogClimberLeftTemp.append(getClimberTemp(1), timeNow);
      dLogClimberLeftCurrent.append(getClimberAmps(1), timeNow);
      dLogClimberLeftVoltage.append(getClimberVoltage(1), timeNow);
      dLogClimberRightTemp.append(getClimberTemp(2), timeNow);
      dLogClimberRightCurrent.append(getClimberAmps(2), timeNow);
      dLogClimberRightVoltage.append(getClimberVoltage(2), timeNow);
    }
  }

  private void checkForFaultConditions() {
    // TODO add fault conditions
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();
      updateLog(false);
    }

    // Stop each side independently based on its own limits.
    if (isSideLimitHitForSetpoint(ClimberConstants.SIDE_LEFT, setpointPercentLeft)) {
      stopClimberMotor(ClimberConstants.SIDE_LEFT);
    }
    if (isSideLimitHitForSetpoint(ClimberConstants.SIDE_RIGHT, setpointPercentRight)) {
      stopClimberMotor(ClimberConstants.SIDE_RIGHT);
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Climber L LOWER", !lowerLimitLeft.get());
      SmartDashboard.putBoolean("Climber R LOWER", !lowerLimitRight.get());
      SmartDashboard.putBoolean("Climber L UPPER", !upperLimitLeft.get());
      SmartDashboard.putBoolean("Climber R UPPER", !upperLimitRight.get());
      SmartDashboard.putNumber("Climber L Temp", getClimberTemp(1));
      SmartDashboard.putNumber("Climber R Temp", getClimberTemp(2));
    }
  }
}
