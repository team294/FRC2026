// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;

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

  // Status signals for the left climber motor
  private final StatusSignal<Temperature> climberLeftTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> climberLeftStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> climberLeftVoltage;                  // Motor voltage, in volts

  // Status signals for the left climber motor
  private final StatusSignal<Temperature> climberRightTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> climberRightStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> climberRightVoltage;                  // Motor voltage, in volts

  private VoltageOut climberVoltageControl = new VoltageOut(0.0);

    // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogClimberLeftTemp = new DoubleLogEntry(log, "/ClimberLeft/Temperature");
  private final DoubleLogEntry dLogClimberLeftCurrent = new DoubleLogEntry(log, "/ClimberLeft/Current");
  private final DoubleLogEntry dLogClimberLeftVoltage = new DoubleLogEntry(log, "/ClimberLeft/Voltage");
  private final DoubleLogEntry dLogClimberRightTemp = new DoubleLogEntry(log, "/ClimberRight/Temperature");
  private final DoubleLogEntry dLogClimberRightCurrent = new DoubleLogEntry(log, "/ClimberRight/Current");
  private final DoubleLogEntry dLogClimberRightVoltage = new DoubleLogEntry(log, "/ClimberRight/Voltage");

  private double setpointPercent; // Current setpoint percent output

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

    // Configure the motor
    climberLeftConfig = new TalonFXConfiguration();
    climberLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberLeftConfig.Voltage.PeakForwardVoltage = UptakeConstants.compensationVoltage;
    climberLeftConfig.Voltage.PeakReverseVoltage = -UptakeConstants.compensationVoltage;
    climberLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    climberLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is limited.
    // This is configured to prevent the breakers from tripping.
    climberLeftConfig.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Lower limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;   // Threshold time, in seconds
    climberLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms to apply.
    climberLeftConfigurator.apply(climberLeftConfig);
    // Inversion configuration inverted by setControl for the follower motor
    // If motor 2 isn't being used as a follower, it will require a separate motor configuration
    climberRightConfigurator.apply(climberLeftConfig);

    // Make right motor follow left motor
    climberRightMotor.setControl(new Follower(climberLeftMotor.getDeviceID(),  MotorAlignmentValue.Opposed)); 

    // Stop the motor
    stopClimberMotors();

    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);

  }

  /**
   * Sets the percent output of the Climber motors using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
   */
  public void setClimberPercentOutput(double percent) {
    setpointPercent = percent;
    climberLeftMotor.setControl(climberVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
   * Stops the Climber motor.
   */
  public void stopClimberMotors() {
    setClimberPercentOutput(0);
  }

  /**
   * Gets the setpoint percent output to the climber motors.
   * @return percent output
   */
  public double getShooterSetpointPercent() {
    return setpointPercent;
  }

  /**
   * Gets the temperature of the Climber motor
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return shooter temperature, in degrees celsius
   */
  public double getShooterTemp(int motorNumber) {
    return motorNumber == 1 ? climberLeftTemp.refresh().getValueAsDouble() : climberRightTemp.refresh().getValueAsDouble();
  }

  /**
   * Gets the stator current of the Climber motor.
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return motor current, in amps
   */
  public double getShooterAmps(int motorNumber) {
    return motorNumber == 1 ? climberLeftStatorCurrent.refresh().getValueAsDouble() : climberRightStatorCurrent.refresh().getValueAsDouble();
  }

  /**
   * Reads the voltage applied to the Climber motor.
   * @param motorNumber motor 1 (left) or 2 (right)
   * @return shooter voltage, in volts
   */
  public double getShooterVoltage(int motorNumber) {
    return motorNumber == 1 ? climberLeftVoltage.refresh().getValueAsDouble() : climberRightVoltage.refresh().getValueAsDouble();
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

      dLogClimberLeftTemp.append(getShooterTemp(1), timeNow);
      dLogClimberLeftCurrent.append(getShooterAmps(1), timeNow);
      dLogClimberLeftVoltage.append(getShooterVoltage(1), timeNow);
      dLogClimberRightTemp.append(getShooterTemp(2), timeNow);
      dLogClimberRightCurrent.append(getShooterAmps(2), timeNow);
      dLogClimberRightVoltage.append(getShooterVoltage(2), timeNow);
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

    // TODO prevent going up if top is read? idk if we will want to override ever
    // TODO prevent going down if bottom is read? idk if we will want to override ever

    // TODO add dashboard values
  }
}
