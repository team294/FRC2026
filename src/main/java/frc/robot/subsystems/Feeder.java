// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANRangeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.StringUtil;
import frc.robot.utilities.Wait;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class Feeder extends SubsystemBase implements Loggable {
  private int logRotationKey;           // Key to identify the log rotation
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follows normal logging cycles
  private String subsystemName;         // Used in file logging and SmartDashboard for naming log entries

  

  // Motors
  private final TalonFX uptakeMotor = new TalonFX(Ports.CANFeederUptakeMotor);
  private final TalonFX spindexerMotor = new TalonFX(Ports.CANFeederSpindexerMotor);

  // CANranges
  private final CANrange CANRangeEntry = new CANrange(Ports.CANRangeEntry); // CANrange located on the feeder's entry point (adjacent to hopper)
  private final CANrange CANRangeExit = new CANrange(Ports.CANRangeExit);   // CANrange located on the feeder's exit point (into shooter)
  
  // CANrange configurators
  private final CANrangeConfigurator CANRangeEntryConfiguator = CANRangeEntry.getConfigurator();
  private final CANrangeConfigurator CANRangeExitConfiguator = CANRangeExit.getConfigurator();

  // CANrange configurations
  private final CANrangeConfiguration CANRangeConfig; // Only need one configuration for both CANranges

  // Status signals for the CANranges
  private final StatusSignal<Distance> CANRangeEntryDistance;
  private final StatusSignal<Distance> CANRangeExitDistance;
  
  private final StatusSignal<Double> CANRangeEntryAmbientSignal;
  private final StatusSignal<Double> CANRangeExitAmbientSignal;

  private final StatusSignal<Boolean> CANRangeEntryIsDetected;
  private final StatusSignal<Boolean> CANRangeExitIsDetected;

  private boolean lastDetectedState = false; // Previous detected state from CANRAnge isDetected() method

  // Configurators
  private final TalonFXConfigurator uptakeConfigurator = uptakeMotor.getConfigurator();
  private final TalonFXConfigurator spindexerConfigurator = spindexerMotor.getConfigurator();

  // Configurations
  private TalonFXConfiguration uptakeConfig;
  private TalonFXConfiguration spindexerConfig;

  // Status signals for the uptake motor
  private final StatusSignal<Temperature> uptakeTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> uptakeStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> uptakeVoltage;                  // Motor voltage, in volts
  private final StatusSignal<AngularVelocity> uptakeEncoderVelocity;  // Motor velocity measured by the encoder, in rotations/second

  // Status signals for the spindexer motor
  private final StatusSignal<Temperature> spindexerTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> spindexerStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> spindexerVoltage;                  // Motor voltage, in volts
  private final StatusSignal<AngularVelocity> spindexerEncoderVelocity;  // Motor velocity measured by the encoder, in rotations/second

  // Motor control modes
  private VoltageOut uptakeVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VoltageOut spindexerVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VelocityVoltage uptakeVelocityControl = new VelocityVoltage(0.0).withEnableFOC(false);
  private VelocityVoltage spindexerVelocityControl = new VelocityVoltage(0.0).withEnableFOC(false);

  // DataLog
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogUptakeTemp = new DoubleLogEntry(log, "/Uptake/Temperature");
  private final DoubleLogEntry dLogUptakeCurrent = new DoubleLogEntry(log, "/Uptake/Current");
  private final DoubleLogEntry dLogUptakeVelocity = new DoubleLogEntry(log, "/Uptake/Velocity");
  private final DoubleLogEntry dLogUptakeVoltage = new DoubleLogEntry(log, "/Uptake/Voltage");
  private final DoubleLogEntry dLogUptakeSetpointRPM = new DoubleLogEntry(log, "/Uptake/SetpointRPM");
  private final DoubleLogEntry dLogSpindexerTemp = new DoubleLogEntry(log, "/Spindexer/Temperature");
  private final DoubleLogEntry dLogSpindexerCurrent = new DoubleLogEntry(log, "/Spindexer/Current");
  private final DoubleLogEntry dLogSpindexerVelocity = new DoubleLogEntry(log, "/Spindexer/Velocity");
  private final DoubleLogEntry dLogSpindexerVoltage = new DoubleLogEntry(log, "/Spindexer/Voltage");
  private final DoubleLogEntry dLogSpindexerSetpointRPM = new DoubleLogEntry(log, "/Spindexer/SetpointRPM");

  private final DoubleLogEntry dLogDistanceEntry = new DoubleLogEntry(log, "/CANRangeEntry/Distance");
  private final DoubleLogEntry dLogAmbientSignalEntry = new DoubleLogEntry(log, "/CANRangeEntry/AmbientSignal");
  private final BooleanLogEntry dLogIsDetectedEntry = new BooleanLogEntry(log, "/CANRangeEntry/IsDetected");

  private final DoubleLogEntry dLogDistanceExit = new DoubleLogEntry(log, "/CANRangeExit/Distance");
  private final DoubleLogEntry dLogAmbientSignalExit = new DoubleLogEntry(log, "/CANRangeExit/AmbientSignal");
  private final BooleanLogEntry dLogIsDetectedExit = new BooleanLogEntry(log, "/CANRangeExit/IsDetected");
  
  private double uptakeSetpointPercent;       // Current setpoint percent output for the uptake
  private double uptakeSetpointRPM = 0.0;     // Current setpoint RPM for the uptake
  private double spindexerSetpointPercent;    // Current setpoint percent output for the spindexer
  private double spindexerSetpointRPM = 0.0;  // Current setpoint RPM for the spindexer

  private double uptakeRPM = 0.0;     // Measured velocity, in RPM
  private double spindexerRPM = 0.0;  // Measured velocity, in RPM

  // Sticky Faults
  private boolean uptakeMissingStickyFault;
  private boolean spindexerMissingStickyFault;
  private boolean CANRangeEntryMissingStickyFault;
  private boolean CANRangeExitMissingStickyFault;
  
  
  public Feeder(String subsystemName) {
    this.subsystemName = subsystemName;
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Get status signals for CANRanges
    CANRangeEntryDistance = CANRangeEntry.getDistance();
    CANRangeExitDistance = CANRangeExit.getDistance();

    CANRangeEntryAmbientSignal = CANRangeEntry.getAmbientSignal();
    CANRangeExitAmbientSignal = CANRangeExit.getAmbientSignal();

    CANRangeEntryIsDetected = CANRangeEntry.getIsDetected();
    CANRangeExitIsDetected = CANRangeExit.getIsDetected();

    // Configure CANRanges
    CANRangeConfig = new CANrangeConfiguration();
    CANRangeConfig.ProximityParams.ProximityThreshold = CANRangeConstants.proximityThreshold;
    CANRangeConfig.ProximityParams.ProximityHysteresis = CANRangeConstants.maxDistanceError;
    CANRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    CANRangeEntryConfiguator.apply(CANRangeConfig);
    CANRangeExitConfiguator.apply(CANRangeConfig);



    // Get status signals for uptake motor
    uptakeTemp = uptakeMotor.getDeviceTemp();
    uptakeStatorCurrent = uptakeMotor.getStatorCurrent();
    uptakeEncoderVelocity = uptakeMotor.getVelocity();
    uptakeVoltage = uptakeMotor.getMotorVoltage();

    // Get status signals for spindexer motor
    spindexerTemp = spindexerMotor.getDeviceTemp();
    spindexerStatorCurrent = spindexerMotor.getStatorCurrent();
    spindexerEncoderVelocity = spindexerMotor.getVelocity();  // Gets angular velocity in rotations per SECOND
    spindexerVoltage = spindexerMotor.getMotorVoltage();

    // Configure the uptake motor
    uptakeConfig = new TalonFXConfiguration();
    uptakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    uptakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    uptakeConfig.Voltage.PeakForwardVoltage = UptakeConstants.compensationVoltage;
    uptakeConfig.Voltage.PeakReverseVoltage = -UptakeConstants.compensationVoltage;
    uptakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    uptakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Configure the spindexer motor
    spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spindexerConfig.Voltage.PeakForwardVoltage = SpindexerConstants.compensationVoltage;
    spindexerConfig.Voltage.PeakReverseVoltage = -SpindexerConstants.compensationVoltage;
    spindexerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;      // Time from 0 to full power, in seconds
    spindexerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;  // Time from 0 to full power, in seconds

    // Prevent the breakers from tripping by limiting current if above the limit for the threshold time
    uptakeConfig.CurrentLimits.SupplyCurrentLimit = 45.0;           // Upper limit for the current, in amps
    uptakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;      // Lower limit for the current, in amps
    uptakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;        // Threshold time, in seconds
    uptakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;     // Enable the limit
    spindexerConfig.CurrentLimits.SupplyCurrentLimit = 45.0;        // Upper limit for the current, in amps
    spindexerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;   // Lower limit for the current, in amps
    spindexerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;     // Threshold time, in seconds
    spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;  // Enable the limit

    // PID terms for the uptake
    uptakeVelocityControl.Slot = 0;
    uptakeConfig.Slot0.kP = UptakeConstants.kP;
    uptakeConfig.Slot0.kI = UptakeConstants.kI;
    uptakeConfig.Slot0.kD = UptakeConstants.kD;
    uptakeConfig.Slot0.kS = UptakeConstants.kS;
    uptakeConfig.Slot0.kV = UptakeConstants.kV;
    uptakeConfig.Slot0.kA = UptakeConstants.kA;

    // PID terms for the spindexer
    spindexerVelocityControl.Slot = 0;
    spindexerConfig.Slot0.kP = SpindexerConstants.kP;
    spindexerConfig.Slot0.kI = SpindexerConstants.kI;
    spindexerConfig.Slot0.kD = SpindexerConstants.kD;
    spindexerConfig.Slot0.kS = SpindexerConstants.kS;
    spindexerConfig.Slot0.kV = SpindexerConstants.kV;
    spindexerConfig.Slot0.kA = SpindexerConstants.kA;

    // Apply the configurations to the uptake motor. This is a blocking call and will wait up to 200ms to apply.
    uptakeConfigurator.apply(uptakeConfig);
    Wait.waitTime(250);

    // Apply the configurations to the spindexer motor. This is a blocking call and will wait up to 200ms to apply.
    spindexerConfigurator.apply(spindexerConfig);
    Wait.waitTime(250);

    // Stop motors
    stopUptake();
    stopSpindexer();

    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);
  }

  /**
   * Sets the percent output of the uptakeMotor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = forward, negative = reverse)
   */
  public void setUptakePercentOutput(double percent) {
    uptakeSetpointPercent = percent;
    uptakeSetpointRPM = 0;
    uptakeMotor.setControl(uptakeVoltageControl.withOutput(percent * UptakeConstants.compensationVoltage));
  }

  /**
   * Sets the percent output of the spindexerMotor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = forward, negative = reverse)
   */
  public void setSpindexerPercentOutput(double percent) {
    spindexerSetpointPercent = percent;
    spindexerSetpointRPM = 0;
    spindexerMotor.setControl(spindexerVoltageControl.withOutput(percent * SpindexerConstants.compensationVoltage));
  }

  /**
   * Stops the uptakeMotor.
   */
  public void stopUptake() {
    setUptakePercentOutput(0);
  }

  /**
   * Stops the spindexerMotor.
   */
  public void stopSpindexer() {
    setSpindexerPercentOutput(0);
  }

  /**
   * Sets the velocity of the uptakeMotor using a closed loop.
   * @param rpm velocity, in RPM
   */
  public void setUptakeVelocity(double rpm) {
    uptakeSetpointRPM = rpm;
    double rps = (rpm / 60.0) * UptakeConstants.uptakeGearRatio;
    uptakeMotor.setControl(uptakeVelocityControl.withVelocity(rps));  
  }

  /**
   * Sets the velocity of the spindexerVelocity using a closed loop.
   * @param rpm velocity, in RPM
   */
  public void setSpindexerVelocity(double rpm) {
    spindexerSetpointRPM = rpm;
    double rps = (rpm / 60.0) * SpindexerConstants.spindexerGearRatio;
    spindexerMotor.setControl(spindexerVelocityControl.withVelocity(rps));
  }

  /**
   * Sets the PIDSVA terms for the uptakeMotor closed loop velocity control.
   * NOTE: This is a blocking call and will wait up to 200ms to apply.
   * @param kP
   * @param kI
   * @param kD
   * @param kS
   * @param kV
   * @param kA
   */
  public void setUptakeMotorPIDSVA(double kP, double kI, double kD, double kS, double kV, double kA) {
    uptakeConfig.Slot0.kP = kP;
    uptakeConfig.Slot0.kI = kI;
    uptakeConfig.Slot0.kD = kD;
    uptakeConfig.Slot0.kS = kS;
    uptakeConfig.Slot0.kV = kV;
    uptakeConfig.Slot0.kA = kA;

    uptakeConfigurator.apply(uptakeConfig);

    DataLogUtil.writeMessage(subsystemName, ": setUptakeMotorPIDSVA, ", StringUtil.buildString("kP: " + kP + ", kI: " + kI + ", kD: " + kD + ", kS: " + kS + ", kV: " + kV + ", kA:" + kA));
  }

  /**
   * Sets the PIDSVA terms for the spindexerMotor closed loop velocity control.
   * NOTE: This is a blocking call and will wait up to 200ms to apply.
   * @param kP
   * @param kI
   * @param kD
   * @param kS
   * @param kV
   * @param kA
   */
  public void setSpindexerPIDSVA(double kP, double kI, double kD, double kS, double kV, double kA)  {
    spindexerConfig.Slot0.kP = kP;
		spindexerConfig.Slot0.kI = kI;
		spindexerConfig.Slot0.kD = kD;
		spindexerConfig.Slot0.kS = kS;
		spindexerConfig.Slot0.kV = kV;
		spindexerConfig.Slot0.kA = kA;

    uptakeConfigurator.apply(uptakeConfig);

    DataLogUtil.writeMessage(subsystemName, ": setSpindexerPIDSVA, ", StringUtil.buildString("kP: " + kP + ", kI: " + kI + ", kD: " + kD + ", kS: " + kS + ", kV: " + kV + ", kA:" + kA));
  }

  /** 
   * Gets the current setpoint percent output for the uptakeMotor.
   * @return setpoint percent, -1.0 to 1.0
   */
  public double getUptakeSetpointPercent() {
    return uptakeSetpointPercent;
  }

  /** 
   * Gets the current setpoint percent output for the spindexerMotor.
   * @return setpoint percent, -1.0 to 1.0
   */
  public double getSpindexerSetpointPercent() {
    return spindexerSetpointPercent;
  }

  /**
   * Gets the velocity of the uptakeMotor.
   * @return motor velocity, in RPM
   */
  public double getUptakeMotorVelocity() {
    uptakeEncoderVelocity.refresh();
    return uptakeEncoderVelocity.getValueAsDouble() * 60.0 / UptakeConstants.uptakeGearRatio;
  }

 /**
   * Gets the velocity of the spindexerMotor.
   * @return motor velocity, in RPM
   */
  public double getSpindexerVelocity() {
    spindexerEncoderVelocity.refresh();
    return spindexerEncoderVelocity.getValueAsDouble() * 60.0 / SpindexerConstants.spindexerGearRatio;
  }

  /**
   * Gets the current of the uptakeMotor.
   * @return motor current, in amps
   */
  public double getUptakeAmps() {
    uptakeStatorCurrent.refresh();
    return uptakeStatorCurrent.getValueAsDouble();
  }

  /**
   * Gets the current of the spindexerMotor.
   * @return motor current, in amps
   */
  public double getSpindexerAmps() {
    spindexerStatorCurrent.refresh();
    return spindexerStatorCurrent.getValueAsDouble();
  }

  /**
   * Gets the error of the PID for uptakeMotor velocity control.
   * @return error, in RPM (positive = measured speed is too fast, negative = too slow)
   */
  public double getUptakeVelocityPIDError() {
    return uptakeRPM - uptakeSetpointRPM;
  }

  /**
   * Gets the error of the PID for spindexerMotor velocity control.
   * @return error, in RPM (positive = measured speed is too fast, negative = too slow)
   */
  public double getSpindexerVelocityPIDError() {
    return spindexerRPM - spindexerSetpointRPM;
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * 0 returns ambient signal from exit CANRange; 1 returns ambient signal from entry CANRange.
   * If number is not 0 or 1, defaults to entry signal.
   * @param CANRange int, 0 = Exit CANRange, 1 = Entry CANRange
   * @return double, ambient signal detected by CANRange
   */
  public double getAmbientSignal(int CANRange) {
    if(CANRange == 0) return CANRangeExitAmbientSignal.refresh().getValueAsDouble(); // If perameter is 0, return exit ambient signal as a double
    
    return CANRangeEntryAmbientSignal.refresh().getValueAsDouble(); // Otherwise return entry signal as a double
    
  }

  /**
   * 0 returns distance detected (in meters) of closest object from exit CANRange; 1 returns distance of closest object from entry CANRange
   * @param CANRange int, 0 = Exit CANRange, 1 = Entry CANRange
   * @return double, distance in meters
   */
  public double getDistance(int CANRange) {
    if(CANRange == 0) return CANRangeExitDistance.refresh().getValueAsDouble();

    return CANRangeEntryDistance.refresh().getValueAsDouble();
  }

  /**
   * Returns true if a ball is within detection range. If state goes from false to true, increments the amount of balls shot by 1
   * Otherwise, if the previous detected state differs from the one currently read, assign value of current to previous.
   * ballsShot will only update when reading from the EXIT CANRange (pass in 0).
   * @param CANRange int, 0 = Exit CANRange, 1 = Entry CANRange
   * @return boolean, true = ball detected, false = no ball detected
   */
  public boolean isDetected(int CANRange) {
    boolean currentDetectedState;
    if(CANRange == 0) currentDetectedState = CANRangeExitIsDetected.refresh().getValue();
    else currentDetectedState = CANRangeEntryIsDetected.refresh().getValue();

    if (lastDetectedState == false && currentDetectedState == true) {
      lastDetectedState = true;
    }
    if (lastDetectedState != currentDetectedState) {
      lastDetectedState = currentDetectedState;
    }
    return currentDetectedState;

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
   * Writes information about the subsystem to the file log.
   * @param logWhenDisabled true = log always, false = only log when enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (!logWhenDisabled && DriverStation.isDisabled()) return;

    long timeNow = RobotController.getFPGATime();

    dLogUptakeTemp.append(uptakeTemp.refresh().getValueAsDouble(), timeNow);
    dLogUptakeVoltage.append(uptakeVoltage.refresh().getValueAsDouble(), timeNow);
    dLogUptakeCurrent.append(getUptakeAmps(), timeNow);
    dLogUptakeVelocity.append(getUptakeMotorVelocity(), timeNow);
    dLogUptakeSetpointRPM.append(uptakeSetpointRPM, timeNow);

    dLogSpindexerTemp.append(spindexerTemp.refresh().getValueAsDouble(), timeNow);
    dLogSpindexerVoltage.append(spindexerVoltage.refresh().getValueAsDouble(), timeNow);
    dLogSpindexerCurrent.append(getSpindexerAmps(), timeNow);
    dLogSpindexerVelocity.append(getSpindexerVelocity(), timeNow);
    dLogSpindexerSetpointRPM.append(spindexerSetpointRPM, timeNow);

    dLogAmbientSignalEntry.append(getAmbientSignal(1), timeNow);
    dLogDistanceEntry.append(getDistance(1), timeNow);
    dLogIsDetectedEntry.append(isDetected(1), timeNow);

    dLogAmbientSignalExit.append(getAmbientSignal(0), timeNow);
    dLogDistanceExit.append(getDistance(0), timeNow);
    dLogIsDetectedExit.append(isDetected(0), timeNow);
  }

  private void checkForFaultConditions() {
     // Reset recordedStickyFault booleans if the Sticky Faults were cleared
   uptakeMissingStickyFault = uptakeMissingStickyFault && RobotPreferences.isStickyFaultActive();
   spindexerMissingStickyFault = spindexerMissingStickyFault && RobotPreferences.isStickyFaultActive();
   CANRangeEntryMissingStickyFault = CANRangeEntryMissingStickyFault && RobotPreferences.isStickyFaultActive();
   CANRangeExitMissingStickyFault = CANRangeExitMissingStickyFault  && RobotPreferences.isStickyFaultActive();

  // If we haven't recorded the sticky fault yet, then check fault condition
  // Uptake motor
    if(!uptakeMissingStickyFault && !uptakeMotor.isConnected()) {
      RobotPreferences.recordStickyFaults("Feeder");
      DataLogUtil.writeMessageEcho("STICKY FAULT Uptake motor disconnected");
      uptakeMissingStickyFault = true;
    }
    // Spindexer motor
    if(!spindexerMissingStickyFault && !spindexerMotor.isConnected()) {
      RobotPreferences.recordStickyFaults("Feeder");
      DataLogUtil.writeMessageEcho("STICKY FAULT Spindexer motor disconnected");
      spindexerMissingStickyFault = true;
    }

    if(!CANRangeEntryMissingStickyFault && !CANRangeEntry.isConnected()) {
      RobotPreferences.recordStickyFaults("Feeder");
      DataLogUtil.writeMessageEcho("STICKY FAULT CANRange1 (Entry) motor disconnected");
      CANRangeEntryMissingStickyFault = true;
    }

    if(!CANRangeExitMissingStickyFault && !CANRangeExit.isConnected()) {
      RobotPreferences.recordStickyFaults("Feeder");
      DataLogUtil.writeMessageEcho("STICKY FAULT CANRange2 (Exit) motor disconnected");
      CANRangeExitMissingStickyFault = true;
    }

    

  }

  @Override
  public void periodic() {
    uptakeRPM = getUptakeMotorVelocity();
    spindexerRPM = getSpindexerVelocity();

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();

      SmartDashboard.putNumber(UPTAKE_RPM.value, getUptakeMotorVelocity());
      SmartDashboard.putNumber(UPTAKE_TARGET_RPM.value, uptakeSetpointRPM);
      SmartDashboard.putNumber(UPTAKE_TEMP.value, uptakeTemp.refresh().getValueAsDouble());

      SmartDashboard.putNumber(SPINDEXER_RPM.value, getSpindexerVelocity());
      SmartDashboard.putNumber(SPINDEXER_TARGET_RPM.value, spindexerSetpointRPM);
      SmartDashboard.putNumber(SPINDEXER_TEMP.value, spindexerTemp.refresh().getValueAsDouble());


      SmartDashboard.putNumber("Uptake Entry Detected Distance", getDistance(1));
      SmartDashboard.putBoolean("Uptake Entry Triggered?", isDetected(1));
      SmartDashboard.putNumber("Uptake Exit Detected Distance", getDistance(0));
      SmartDashboard.putBoolean("Uptake Exit Triggered?", isDetected(0));
    }    
  }
}
