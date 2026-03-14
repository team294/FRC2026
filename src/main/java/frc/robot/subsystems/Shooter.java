// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.StringUtil;

public class Shooter extends SubsystemBase implements Loggable {
  private final int logRotationKey;     // Key to identify the log rotation
  private boolean fastLogging = true;  // true = log every cycle, false = log every 10 cycles
  private String subsystemName;         // Name of the subsystem, used for logging/dashboard

  // Motor, configurator, and feedforward control objects
  private final TalonFX shooterMotor1 = new TalonFX(Ports.CANShooterMotor1);
  private final TalonFX shooterMotor2 = new TalonFX(Ports.CANShooterMotor2);
  private final TalonFXConfigurator shooterMotor1Configurator;
  private final TalonFXConfigurator shooterMotor2Configurator;
  private TalonFXConfiguration shooterMotorConfiguration;

  // Live data reported by the motor controller
  private final StatusSignal<Temperature> shooter1Temperature;         // Temperature, in degrees Celsius
  private final StatusSignal<Current> shooter1Current;                 // Stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> shooter1Voltage;                 // Voltage, in volts
  private final StatusSignal<Angle> shooter1EncoderPosition;           // Encoder position, in rotations
  private final StatusSignal<AngularVelocity> shooter1EncoderVelocity; // Encoder velocity, in rotations/second
  private final StatusSignal<Voltage> shooter1SupplyVoltage;           // Incoming bus voltage in volts
  private final StatusSignal<Temperature> shooter2Temperature;         // Temperature, in degrees Celsius
  private final StatusSignal<Current> shooter2Current;                 // Stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> shooter2Voltage;                 // Voltage, in volts
  private final StatusSignal<Angle> shooter2EncoderPosition;           // Encoder position, in rotations
  private final StatusSignal<AngularVelocity> shooter2EncoderVelocity; // Encoder velocity, in rotations/second
  private final StatusSignal<Voltage> shooter2SupplyVoltage;           // Incoming bus voltage in volts

  // Voltage and velocity controls
  private VoltageOut shooterVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0);

  private boolean velocityControlOn = false;  // true = velocity control is on, false = velocity control is off (e.g. percent output, position)
  private double setpointRPM;                 // Target velocity, in RPM (positive = forward, negative = reverse)

  // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTemp1 = new DoubleLogEntry(log, "/Shooter/Temperature1");
  private final DoubleLogEntry dLogCurrent1 = new DoubleLogEntry(log, "/Shooter/Current1");
  private final DoubleLogEntry dLogVelocity1 = new DoubleLogEntry(log, "/Shooter/Velocity1");
  private final DoubleLogEntry dLogPosition1 = new DoubleLogEntry(log, "/Shooter/Position1");
  private final DoubleLogEntry dLogVoltage1 = new DoubleLogEntry(log, "/Shooter/Voltage1");
  private final DoubleLogEntry dLogSupplyVoltage1 = new DoubleLogEntry(log, "/Shooter/SupplyVoltage1");
  private final DoubleLogEntry dLogTemp2 = new DoubleLogEntry(log, "/Shooter/Temperature2");
  private final DoubleLogEntry dLogCurrent2 = new DoubleLogEntry(log, "/Shooter/Current2");
  private final DoubleLogEntry dLogVelocity2 = new DoubleLogEntry(log, "/Shooter/Velocity2");
  private final DoubleLogEntry dLogPosition2 = new DoubleLogEntry(log, "/Shooter/Position2");
  private final DoubleLogEntry dLogVoltage2 = new DoubleLogEntry(log, "/Shooter/Voltage2");
  private final DoubleLogEntry dLogSupplyVoltage2 = new DoubleLogEntry(log, "/Shooter/SupplyVoltage2");
  private final DoubleLogEntry dLogVelocitySet = new DoubleLogEntry(log, "/Shooter/VelocitySet");

  private final BooleanLogEntry dLogVelocityControl = new BooleanLogEntry(log, "/Shooter/VelocityControlOn");

  // Sticky Faults
  private boolean recordedMotor1MissingStickyFault;
  private boolean recordedMotor2MissingStickyFault;

  public Shooter(String subsystemName) {
    // Set the subsystem name
    this.subsystemName = subsystemName;
    
    // Allocate a log rotation key
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Get the motor configurator
    shooterMotor1Configurator = shooterMotor1.getConfigurator();
    shooterMotor2Configurator = shooterMotor2.getConfigurator();

    // Get signal and sensor objects
    shooter1Temperature = shooterMotor1.getDeviceTemp();
    shooter1Current = shooterMotor1.getStatorCurrent();
    shooter1EncoderVelocity = shooterMotor1.getVelocity();
    shooter1EncoderPosition = shooterMotor1.getPosition();
    shooter1Voltage = shooterMotor1.getMotorVoltage();
    shooter1SupplyVoltage = shooterMotor1.getSupplyVoltage();
    shooter2Temperature = shooterMotor2.getDeviceTemp();
    shooter2Current = shooterMotor2.getStatorCurrent();
    shooter2EncoderVelocity = shooterMotor2.getVelocity();
    shooter2EncoderPosition = shooterMotor2.getPosition();
    shooter2Voltage = shooterMotor2.getMotorVoltage();
    shooter2SupplyVoltage = shooterMotor2.getSupplyVoltage();

    // Configure the motor
    shooterMotorConfiguration = new TalonFXConfiguration();
    shooterMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterMotorConfiguration.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
    shooterMotorConfiguration.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
    shooterMotorConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;      // Time from 0 to full power in open loop, in seconds
    shooterMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;  // Time from 0 to full power in closed loop, in seconds

    // If the current is above the supply current limit for the threshold time, the current is limited.
    // This is configured to prevent the breakers from tripping.
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // Lower limit for the current, in amps
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.2;   // Threshold time, in seconds
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set PID coefficients
    shooterVelocityControl.Slot = 0;

    shooterVelocityControl.OverrideBrakeDurNeutral = false;
    shooterMotorConfiguration.Slot0.kP = ShooterConstants.ShooterkP;
    shooterMotorConfiguration.Slot0.kI = ShooterConstants.ShooterkI;
    shooterMotorConfiguration.Slot0.kD = ShooterConstants.ShooterkD;
    shooterMotorConfiguration.Slot0.kS = ShooterConstants.ShooterkS;
    shooterMotorConfiguration.Slot0.kV = ShooterConstants.ShooterkV;
    shooterMotorConfiguration.Slot0.kA = ShooterConstants.ShooterkA;

    // Create the simple feedforward model
    //shooterMotorFeedforward = new SimpleMotorFeedforward(ShooterConstants.ShooterkS, ShooterConstants.ShooterkV, ShooterConstants.ShooterkA);

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms to apply.
    shooterMotor1Configurator.apply(shooterMotorConfiguration);
    // Inversion configuration inverted by setControl for the follower motor
    // If motor 2 isn't being used as a follower, it will require a separate motor configuration
    shooterMotor2Configurator.apply(shooterMotorConfiguration);

    // Make motor2 follow motor1
    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(),  MotorAlignmentValue.Opposed)); 

    // Stop the motor
    stopShooterMotor();

    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);
  }

  /**
   * Sets the percent output of the Shooter motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
   */
  public void setShooterPercentOutput(double percent) {
    velocityControlOn = false;
    setpointRPM = 0.0;
    shooterMotor1.setControl(shooterVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
   * Sets the velocity of the Shooter wheel using voltage compensation. 
   * @param velocityRPM the velocity, in rpm, to set the wheel to (positive = shoot, negative = reverse)
   */
  public void setShooterVelocity(double velocityRPM) {
    velocityControlOn = true;
    setpointRPM = velocityRPM;
    double motorRPS = (velocityRPM / 60.0) / ShooterConstants.gearRatio;

    shooterMotor1.setControl(
      shooterVelocityControl
        .withVelocity(motorRPS)
      );
  }

  /**
   * Stops the Shooter motor.
   */
  public void stopShooterMotor() {
    setShooterPercentOutput(0);
  }

  /**
   * Gets the velocity of the Shooter wheel.
   * @param motorNumber int which motor, 1 or 2, to get velocity for
   * @return wheel velocity, in RPM
   */
  public double getShooterVelocity(int motorNumber) {
    return motorNumber == 1 ? shooter1EncoderVelocity.refresh().getValueAsDouble() * ShooterConstants.gearRatio * 60.0
                            : shooter2EncoderVelocity.refresh().getValueAsDouble() * ShooterConstants.gearRatio * 60.0;
  }

  /**
   * Returns the shooter wheel position
   * @param motorNumber int which motor, 1 or 2, to get position for
   * @return position of shooter in wheel rotations
   */
  public double getShooterEncoderPosition(int motorNumber) {
    return motorNumber == 1 ? shooter1EncoderPosition.refresh().getValueAsDouble() * ShooterConstants.gearRatio 
                            : shooter2EncoderPosition.refresh().getValueAsDouble() * ShooterConstants.gearRatio;
  }

  /**
   * Gets the current of the Shooter motor.
   * @param motorNumber int which motor, 1 or 2, to get amps for
   * @return motor current, in amps
   */
  public double getShooterAmps(int motorNumber) {
    return motorNumber == 1 ? shooter1Current.refresh().getValueAsDouble() : shooter2Current.refresh().getValueAsDouble();
  }

  /**
   * Reads the shooter voltage applied to the motor
   * @param motorNumber int which motor, 1 or 2, to get volts for
   * @return shooter voltage, in volts
   */
  public double getShooterVoltage(int motorNumber) {
    return motorNumber == 1 ? shooter1Voltage.refresh().getValueAsDouble() : shooter2Voltage.refresh().getValueAsDouble();
  }

  /**
   * Reads the shooter temperature
   * @param motorNumber int which motor, 1 or 2, to get temperature for
   * @return shooter temperature, in degrees celsius
   */
  public double getShooterTemp(int motorNumber) {
    return motorNumber == 1 ? shooter1Temperature.refresh().getValueAsDouble() : shooter2Temperature.refresh().getValueAsDouble();
  }

    /**
   * Returns whether velocity control is on
   * @return true if velocity control is on. false if motor stopped or motor under percent control
   */
  public boolean isVelocityControlOn(){
    return velocityControlOn;
  }

  /**
   * Returns the shooter wheel setpoint in RPM
   * @return the shooter wheel setpoint in rotations per minute
   */
  public double getShooterSetpointRPM() {
    return setpointRPM;
  }

  /**
   * Sets PIDSVA feedforward and feedback parameters for the shooter motor.
   * <p><b> This is a blocking call and will wait up to 200ms to apply. </b>
   * @param kP volts / error in motor rps
   * @param kI
   * @param kD
   * @param kS volts to overcome static friction
   * @param kV volts / desired motor rps
   * @param kA
   */
  public void setPIDSVA(double kP, double kI, double kD, double kS, double kV, double kA) {
    shooterMotorConfiguration.Slot0.kP = kP;
    shooterMotorConfiguration.Slot0.kI = kI;
    shooterMotorConfiguration.Slot0.kD = kD;
    shooterMotorConfiguration.Slot0.kS = kS;
    shooterMotorConfiguration.Slot0.kV = kV;
    shooterMotorConfiguration.Slot0.kA = kA;

    shooterMotor1Configurator.apply(shooterMotorConfiguration);

    String s = StringUtil.buildString("kP: " + kP + ", kI: " + kI + ", kD: " + kD + ", kS: " + kS + ", kV: " + kV + ", kA:" + kA);
    SmartDashboard.putString("Shooter Applied PID", s);
    DataLogUtil.writeMessage(subsystemName, ".setPIDSVA:  ", s);
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about the Shooter to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogTemp1.append(getShooterTemp(1), timeNow);
      dLogVoltage1.append(getShooterVoltage(1), timeNow);
      dLogCurrent1.append(getShooterAmps(1), timeNow);
      dLogVelocity1.append(getShooterVelocity(1), timeNow);
      dLogPosition1.append(getShooterEncoderPosition(1), timeNow);
      dLogSupplyVoltage1.append(shooter1SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogTemp2.append(getShooterTemp(2), timeNow);
      dLogVoltage2.append(getShooterVoltage(2), timeNow);
      dLogCurrent2.append(getShooterAmps(2), timeNow);
      dLogVelocity2.append(getShooterVelocity(2), timeNow);
      dLogPosition2.append(getShooterEncoderPosition(2), timeNow);
      dLogSupplyVoltage2.append(shooter2SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogVelocitySet.append(getShooterSetpointRPM(), timeNow);

      dLogVelocityControl.append(velocityControlOn, timeNow);
    }
  }

  /**
   * Checks for fault conditions and records stick faults if any exist.
   */
  private void checkForFaultConditions() {
    // Reset recordedStickyFault booleans if the Sticky Faults were cleared
    recordedMotor1MissingStickyFault = recordedMotor1MissingStickyFault && RobotPreferences.isStickyFaultActive();
    recordedMotor2MissingStickyFault = recordedMotor2MissingStickyFault && RobotPreferences.isStickyFaultActive();

    // If we haven't recorded the sticky fault yet, then check fault condition
    if(!recordedMotor1MissingStickyFault && !shooterMotor1.isConnected()) {
      RobotPreferences.recordStickyFaults("Shooter-Motor1");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Motor 1 Missing");
      recordedMotor1MissingStickyFault = true;
    }

    if(!recordedMotor2MissingStickyFault && !shooterMotor2.isConnected()) {
      RobotPreferences.recordStickyFaults("Shooter-Motor2");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Motor 2 Missing");
      recordedMotor2MissingStickyFault = true;
    }
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();
      updateLog(false);
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 1 Voltage"), getShooterVoltage(1));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 1 Position Rev"), getShooterEncoderPosition(1));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 1 RPM"), getShooterVelocity(1));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 1 Temp C"), getShooterTemp(1));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 2 Voltage"), getShooterVoltage(2));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 2 Position Rev"), getShooterEncoderPosition(2));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 2 RPM"), getShooterVelocity(2));
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " 2 Temp C"), getShooterTemp(2));

      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Avg RPM"), (getShooterVelocity(1) + getShooterVelocity(2)) / 2.0);

      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Target RPM"), getShooterSetpointRPM());
      SmartDashboard.putBoolean(StringUtil.buildString(subsystemName, " Velocity Control On"), isVelocityControlOn());
    }
  }
}
