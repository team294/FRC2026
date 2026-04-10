// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

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

   // Hood Linear Actuator Servo
  private final TalonFXS hoodMotor1 = new TalonFXS(Ports.CANHoodMotor1);
  private final TalonFXS hoodMotor2 = new TalonFXS(Ports.CANHoodMotor2);
  private final TalonFXSConfigurator hoodMotor1Configurator;
  private final TalonFXSConfigurator hoodMotor2Configurator;
  private TalonFXSConfiguration hoodMotorConfiguration;

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
  
  private final StatusSignal<Voltage> hood1PositionVoltage;
  private final StatusSignal<Voltage> hood1MotorVoltage;
  private final StatusSignal<Voltage> hood1SupplyVoltage;
  private final StatusSignal<Current> hood1Current;
  private final StatusSignal<Voltage> hood2PositionVoltage;
  private final StatusSignal<Voltage> hood2MotorVoltage;
  private final StatusSignal<Voltage> hood2SupplyVoltage;
  private final StatusSignal<Current> hood2Current;

  // Voltage and velocity controls
  private VoltageOut shooterVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VelocityVoltage shooterVelocityControl = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

  private boolean shooterVelocityControlOn = false;  // true = velocity control is on, false = velocity control is off (e.g. percent output, position)
  private double shooterSetpointRPM;                 // Target velocity, in RPM (positive = forward, negative = reverse)
  
  private VoltageOut hood1VoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private VoltageOut hood2VoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private double hoodAngleSetpoint;
  private double hood1VoltageSetpoint;
  private double hood2VoltageSetpoint;
  private boolean hood1PositionControlOn = false;
  private boolean hood2PositionControlOn = false;
  
  private PIDController hood1PidController = new PIDController(ShooterConstants.hood1kP, ShooterConstants.hood1kI, ShooterConstants.hood1kD);
  private PIDController hood2PidController = new PIDController(ShooterConstants.hood2kP, ShooterConstants.hood2kI, ShooterConstants.hood2kD);
  
  // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogShooter1Temp = new DoubleLogEntry(log, "/Shooter/Temperature1");
  private final DoubleLogEntry dLogShooter1Current = new DoubleLogEntry(log, "/Shooter/Current1");
  private final DoubleLogEntry dLogShooter1Velocity = new DoubleLogEntry(log, "/Shooter/Velocity1");
  private final DoubleLogEntry dLogShooter1Position = new DoubleLogEntry(log, "/Shooter/Position1");
  private final DoubleLogEntry dLogShooter1Voltage = new DoubleLogEntry(log, "/Shooter/Voltage1");
  private final DoubleLogEntry dLogShooter1SupplyVoltage = new DoubleLogEntry(log, "/Shooter/SupplyVoltage1");
  private final DoubleLogEntry dLogShooter2Temp = new DoubleLogEntry(log, "/Shooter/Temperature2");
  private final DoubleLogEntry dLogShooter2Current = new DoubleLogEntry(log, "/Shooter/Current2");
  private final DoubleLogEntry dLogShooter2Velocity = new DoubleLogEntry(log, "/Shooter/Velocity2");
  private final DoubleLogEntry dLogShooter2Position = new DoubleLogEntry(log, "/Shooter/Position2");
  private final DoubleLogEntry dLogShooter2Voltage = new DoubleLogEntry(log, "/Shooter/Voltage2");
  private final DoubleLogEntry dLogShooter2SupplyVoltage = new DoubleLogEntry(log, "/Shooter/SupplyVoltage2");
  private final DoubleLogEntry dLogShooterVelocitySet = new DoubleLogEntry(log, "/Shooter/VelocitySet");
  private final BooleanLogEntry dLogShooterVelocityControl = new BooleanLogEntry(log, "/Shooter/VelocityControlOn");

  private final DoubleLogEntry dLogHoodAngle = new DoubleLogEntry(log, "/Hood/Angle");
  private final DoubleLogEntry dLogHood1Angle = new DoubleLogEntry(log, "/Hood/Angle1");
  private final DoubleLogEntry dLogHood1AnalogVoltage = new DoubleLogEntry(log, "/Hood/AnalogVoltage1");
  private final DoubleLogEntry dLogHood1Voltage = new DoubleLogEntry(log, "/Hood/Voltage1");
  private final DoubleLogEntry dLogHood1SupplyVoltage = new DoubleLogEntry(log, "/Hood/SupplyVoltage1");
  private final DoubleLogEntry dLogHood1Current = new DoubleLogEntry(log, "/Hood/Current1");
  private final DoubleLogEntry dLogHood2Angle = new DoubleLogEntry(log, "/Hood/Angle2");
  private final DoubleLogEntry dLogHood2AnalogVoltage = new DoubleLogEntry(log, "/Hood/AnalogVoltage2");
  private final DoubleLogEntry dLogHood2Voltage = new DoubleLogEntry(log, "/Hood/Voltage2");
  private final DoubleLogEntry dLogHood2SupplyVoltage = new DoubleLogEntry(log, "/Hood/SupplyVoltage2");
  private final DoubleLogEntry dLogHood2Current = new DoubleLogEntry(log, "/Hood/Current2");
  private final DoubleLogEntry dLogHoodAngleSetpoint = new DoubleLogEntry(log, "/Hood/AngleSetpoint");
  private final DoubleLogEntry dLogHoodVoltageSetpoint = new DoubleLogEntry(log, "/Hood/VoltageSetpoint");
  private final BooleanLogEntry dLogHoodPositionControlOn = new BooleanLogEntry(log, "/Hood/PositionControlOn");
 

  // Sticky Faults
  private boolean recordedShooterMotor1MissingStickyFault;
  private boolean recordedShooterMotor2MissingStickyFault;

  private boolean recordedHoodMotor1MissingStickyFault;
  private boolean recordedHoodMotor2MissingStickyFault;

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

    hood1PositionVoltage = hoodMotor1.getAnalogVoltage();
    hood1MotorVoltage = hoodMotor1.getMotorVoltage();
    hood1SupplyVoltage = hoodMotor1.getSupplyVoltage();
    hood1Current = hoodMotor1.getStatorCurrent();
    hood2PositionVoltage = hoodMotor2.getAnalogVoltage();
    hood2MotorVoltage = hoodMotor2.getMotorVoltage();
    hood2SupplyVoltage = hoodMotor2.getSupplyVoltage();
    hood2Current = hoodMotor2.getStatorCurrent();

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
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
    shooterMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
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
    
    hoodMotor1Configurator = hoodMotor1.getConfigurator();
    hoodMotor2Configurator = hoodMotor2.getConfigurator();
    hood1PidController.reset();
    hood2PidController.reset();
    hood1PidController.setTolerance(ShooterConstants.hoodPIDErrorTolerance);
    hood2PidController.setTolerance(ShooterConstants.hoodPIDErrorTolerance);

    hoodMotorConfiguration = new TalonFXSConfiguration();
    hoodMotorConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
    hoodMotorConfiguration.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;
    hoodMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodMotorConfiguration.Voltage.PeakForwardVoltage = ShooterConstants.compensationVoltage;
    hoodMotorConfiguration.Voltage.PeakReverseVoltage = -ShooterConstants.compensationVoltage;
    hoodMotorConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;      // Time from 0 to full power in open loop, in seconds
    hoodMotorConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;  // Time from 0 to full power in closed loop, in seconds
    // hoodMotorConfiguration.SoftwareLimitSwitch.

    // If the current is above the supply current limit for the threshold time, the current is limited.
    // This is configured to prevent the breakers from tripping.
    hoodMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
    hoodMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
    hoodMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
    hoodMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
       
    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms to apply.
    shooterMotor1Configurator.apply(shooterMotorConfiguration);
    // Inversion configuration inverted by setControl for the follower motor
    // If motor 2 isn't being used as a follower, it will require a separate motor configuration
    shooterMotor2Configurator.apply(shooterMotorConfiguration);

    hoodMotor1Configurator.apply(hoodMotorConfiguration);
    hoodMotor2Configurator.apply(hoodMotorConfiguration);

    // Make motor2 follow motor1
    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(),  MotorAlignmentValue.Opposed)); 
    // hoodMotor2.setControl(new Follower(hoodMotor1.getDeviceID(), MotorAlignmentValue.Aligned)); controlling both separately
    
    // Stop the motor
    stopShooterMotor();
    setHoodVoltageBothMotors(0);
    
    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);
  }

  // *****  Shooter methods *****

  /**
   * Sets the percent output of the Shooter motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = shoot direction, negative = reverse)
   */
  public void setShooterPercentOutput(double percent) {
    shooterVelocityControlOn = false;
    shooterSetpointRPM = 0.0;
    shooterMotor1.setControl(shooterVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
   * Sets the velocity of the Shooter wheel using voltage compensation. 
   * @param velocityRPM the velocity, in rpm, to set the wheel to (positive = shoot, negative = reverse)
   */
  public void setShooterVelocity(double velocityRPM) {
    shooterVelocityControlOn = true;
    shooterSetpointRPM = velocityRPM;
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
   * Gets the velocity of the shooter wheel, averaging the two motors.
   * @return wheel velocity, in RPM
   */
  public double getShooterVelocity() {
    return (getShooterVelocity(1) + getShooterVelocity(2)) / 2.0;
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
  public boolean isShooterVelocityControlOn(){
    return shooterVelocityControlOn;
  }

  /**
   * Returns the shooter wheel setpoint in RPM
   * @return the shooter wheel setpoint in rotations per minute
   */
  public double getShooterSetpointRPM() {
    return shooterSetpointRPM;
  }

  /**
   * Gets the error of the PID for spindexerMotor velocity control.
   * @return error, in RPM (positive = measured speed is too fast, negative = too slow)
   */
  public double getShooterVelocityPIDError() {
    return getShooterVelocity() - shooterSetpointRPM;
  }

  /**
   * Returns if shooter is within tolerance from the most recent goal velocity
   * @return true = in tolerance, false = not at goal velocity.
   */
  public boolean isShooterAtGoalVelocity() {
    return Math.abs(getShooterVelocityPIDError()) < ShooterConstants.shooterVelocityErrorTolerance && (shooterSetpointRPM != 0.0);
  }

  // *****  Hood methods *****

  /**
   * Reads the hood analog voltage
   * @param motorNumber int which motor, 1 or 2, to get analog voltage for
   * @return hood analog voltage 
   * <p> The voltage of the analog pin (pin 3) of the Talon FXS data port. The analog pin reads a nominal voltage of 0-5V.
   */
  public double getHoodAnalogVoltage(int motorNumber) {
    return motorNumber == 1 ? hood1PositionVoltage.refresh().getValueAsDouble() : hood2PositionVoltage.refresh().getValueAsDouble();
  }

  /**
   * Reads the average analog voltage of both hood motors
   * @return hood analog voltage 
   * <p> The voltage of the analog pin (pin 3) of the Talon FXS data port. The analog pin reads a nominal voltage of 0-5V.
   */
  public double getHoodAnalogVoltage() {
    return (getHoodAnalogVoltage(1) + getHoodAnalogVoltage(2)) / 2.0;
  }

  /**
   * Set the hood voltage to BOTH linear actuators 
   * @param voltage in volts [-12, 12]; positive is up and negative is down
   */
  public void setHoodVoltageBothMotors(double voltage) {
    hood1VoltageSetpoint = voltage;
    hood2VoltageSetpoint = voltage;
    hood1PositionControlOn = false;
    hood2PositionControlOn = false;
    hoodMotor1.setControl(hood1VoltageControl.withOutput(voltage* ShooterConstants.hoodMotorRatio2to1));
    hoodMotor2.setControl(hood2VoltageControl.withOutput(voltage));
  }

  /**
   * Set the hood voltage to ONE linear actuator 
   * @param voltage in volts [-12, 12]; positive is up and negative is down
   */
  public void setHoodVoltageOneMotor(double voltage, int motorNumber) {
    if (motorNumber == 1) {
      hoodMotor1.setControl(hood1VoltageControl.withOutput(voltage));
      hood1VoltageSetpoint = voltage;
      hood1PositionControlOn = false;
    } else {
      hoodMotor2.setControl(hood2VoltageControl.withOutput(voltage));
      hood2VoltageSetpoint = voltage;
      hood2PositionControlOn = false;
    }
  }

  // public void setHoodAngleSetpoint(double angle) {
  //   double safeAngle = MathUtil.clamp(angle, ShooterConstants.hoodMinimumAngle, ShooterConstants.hoodMaximumAngle);
  //   hoodAngleSetpoint = safeAngle;
  //   hood1PositionControlOn = true;
  //   hood2PositionControlOn = true;
  // }

  /**
   * Move both motors with simple PID controller.
   * <p><b>Note:</b> This method needs to be called every execute() cycle to continue using PID control.
   * @param angle angle to set in degrees
   */
  public void setHoodAnglePID(double angle) {
    double safeAngle = MathUtil.clamp(angle, ShooterConstants.hoodMinimumAngle, ShooterConstants.hoodMaximumAngle);
    hoodAngleSetpoint = safeAngle;
    hood1PositionControlOn = true;
    hood2PositionControlOn = true;
  
    if ( Math.abs(getHoodAngle() - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance/2.0 ) {
      // If hood is very close to target position, then turn off motor.
      hoodMotor1.set(0.0);
      hoodMotor2.set(0.0);
    } else {
      // hoodMotor1.set(hood1PidController.calculate(getHoodAngle(1), hoodAngleSetpoint)); 
      // hoodMotor2.set(hood2PidController.calculate(getHoodAngle(2), hoodAngleSetpoint)); 
      //HoodFix
      // Due to error on the left (#1) hood analog sensor, calculate PID based on right hood motor
      // and set motor #1 as a follower with a fixed calibration ratio for motor/friction differences.
      double hoodPct = hood2PidController.calculate(getHoodAngle(2), hoodAngleSetpoint);
      hoodMotor1.set(hoodPct * ShooterConstants.hoodMotorRatio2to1);
      hoodMotor2.set(hoodPct);
    }
  }
  
  /**
   * Get estimated linear actuator length based on one hood motor's analog voltage, INCLUDING BASE OF ACTUATOR.
   * @param motorNumber hood # to estimate
   * @return estimated specified hood motor extension length, in inches
   */
  public double getHoodExtensionLength(int motorNumber) {
    return (motorNumber == 1) ? ShooterConstants.hood1VoltageToExtensionLengthSlope * getHoodAnalogVoltage(1) + ShooterConstants.hoodActuatorMinExtensionLength
    : ShooterConstants.hood2VoltageToExtensionLengthSlope * getHoodAnalogVoltage(2) + ShooterConstants.hoodActuatorMinExtensionLength;
  }

  /**
   * Get estimated hood extension length based on both hood motors' analog voltage.
   * @return average estimated hood motor extension length, in inches
   */
  public double getHoodExtensionLength() {
    return (getHoodExtensionLength(1) + getHoodExtensionLength(2)) / 2.0;
  }

  /**
   * Estimate current hood angle based on one hood motor's current length.
   * @param motorNumber hood # to estimate
   * @return estimated hood angle from specified motor's reading, in degrees
   */
  public double getHoodAngle(int motorNumber) {
    return Math.toDegrees(
      Math.acos(
        ((ShooterConstants.radius1 * ShooterConstants.radius1) + (ShooterConstants.radius2 * ShooterConstants.radius2)
          - (getHoodExtensionLength(motorNumber) * getHoodExtensionLength(motorNumber)))
          / (2.0 * ShooterConstants.radius1 * ShooterConstants.radius2)
      )
    ) - (54.8122837191 - ShooterConstants.hoodMinimumAngle);
  }

  /**
   * Converts a hood angle (deg) to a linear actuator extension (in)
   * @param double hood angle, degrees
   * @return double linear extension of the linear actuator, in inches
   */
  public double getHoodAngleToExtension(double angle) {
    return Math.sqrt(
      (ShooterConstants.radius2 * ShooterConstants.radius2) + (ShooterConstants.radius1 * ShooterConstants.radius1)
        - Math.cos(Math.toRadians(angle + 54.8122837191 - ShooterConstants.hoodMinimumAngle)) * (2 * ShooterConstants.radius1 * ShooterConstants.radius2)
    ) + ShooterConstants.hoodActuatorMinExtensionLength;
  }

  /**
   * This should estimate current hood angle based on both hood motors' current length.
   * However, due to flaky left hood motor, this only returns the value from the right hood motor. 
   * @return estimated hood angle, in degrees
   */
  public double getHoodAngle() {
    return getHoodAngle(2);
    //HoodFix
    // return (getHoodAngle(1) + getHoodAngle(2)) / 2.0;
  }

  /**
   * Returns the hood angle setpoint in degrees
   * @return the hood angle setpoint in degrees
   */
  public double getHoodAngleSetpoint() {
    return hoodAngleSetpoint;
  }

  /**
   * Returns if hood is within tolerance from the most recent goal angle
   * @return true = in tolerance, false = not at goal angle.
   */
  public boolean isHoodAtGoalAngle() {
    return Math.abs(getHoodAngle() - hoodAngleSetpoint) < ShooterConstants.hoodAngleErrorTolerance;
  }

  /**
   * Returns if hood is within tolerance of trench configuration
   * @return true = in tolerance, false = not in trench configuration.
   */
  public boolean isHoodSafeForTrench() {
    return Math.abs(getHoodAngle() - ShooterConstants.hoodMinimumAngle) < ShooterConstants.hoodAngleErrorTolerance;
  }

  // *****  Information methods *****

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

      dLogShooter1Temp.append(getShooterTemp(1), timeNow);
      dLogShooter1Voltage.append(getShooterVoltage(1), timeNow);
      dLogShooter1Current.append(getShooterAmps(1), timeNow);
      dLogShooter1Velocity.append(getShooterVelocity(1), timeNow);
      dLogShooter1Position.append(getShooterEncoderPosition(1), timeNow);
      dLogShooter1SupplyVoltage.append(shooter1SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogShooter2Temp.append(getShooterTemp(2), timeNow);
      dLogShooter2Voltage.append(getShooterVoltage(2), timeNow);
      dLogShooter2Current.append(getShooterAmps(2), timeNow);
      dLogShooter2Velocity.append(getShooterVelocity(2), timeNow);
      dLogShooter2Position.append(getShooterEncoderPosition(2), timeNow);
      dLogShooter2SupplyVoltage.append(shooter2SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogShooterVelocitySet.append(getShooterSetpointRPM(), timeNow);
      dLogShooterVelocityControl.append(shooterVelocityControlOn, timeNow);

      dLogHoodAngle.append(getHoodAngle(), timeNow);
      dLogHood1Angle.append(getHoodAngle(1), timeNow);
      dLogHood1AnalogVoltage.append(getHoodAnalogVoltage(1), timeNow);
      dLogHood1Voltage.append(hood1MotorVoltage.refresh().getValueAsDouble(), timeNow);
      dLogHood1SupplyVoltage.append(hood1SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogHood1Current.append(hood1Current.refresh().getValueAsDouble(), timeNow);
      dLogHood2Angle.append(getHoodAngle(2), timeNow);
      dLogHood2AnalogVoltage.append(getHoodAnalogVoltage(2), timeNow);
      dLogHood2Voltage.append(hood2MotorVoltage.refresh().getValueAsDouble(), timeNow);
      dLogHood2SupplyVoltage.append(hood2SupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogHood2Current.append(hood2Current.refresh().getValueAsDouble(), timeNow);
      dLogHoodAngleSetpoint.append(hoodAngleSetpoint, timeNow);
      dLogHoodVoltageSetpoint.append(hood1VoltageSetpoint, timeNow);
      dLogHoodVoltageSetpoint.append(hood2VoltageSetpoint, timeNow);
      dLogHoodPositionControlOn.append(hood2PositionControlOn, timeNow);
      dLogHoodPositionControlOn.append(hood2PositionControlOn, timeNow);
    }
  }

  /**
   * Checks for fault conditions and records stick faults if any exist.
   */
  private void checkForFaultConditions() {
    // Reset recordedStickyFault booleans if the Sticky Faults were cleared
    recordedShooterMotor1MissingStickyFault = recordedShooterMotor1MissingStickyFault && RobotPreferences.isStickyFaultActive();
    recordedShooterMotor2MissingStickyFault = recordedShooterMotor2MissingStickyFault && RobotPreferences.isStickyFaultActive();
    
    recordedHoodMotor1MissingStickyFault = recordedHoodMotor1MissingStickyFault && RobotPreferences.isStickyFaultActive();
    recordedHoodMotor2MissingStickyFault = recordedHoodMotor2MissingStickyFault && RobotPreferences.isStickyFaultActive();

    // If we haven't recorded the sticky fault yet, then check fault condition
    if(!recordedShooterMotor1MissingStickyFault && !shooterMotor1.isConnected()) {
      RobotPreferences.recordStickyFaults("Shooter-Motor1");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Shooter Motor 1 Missing");
      recordedShooterMotor1MissingStickyFault = true;
    }

    if(!recordedShooterMotor2MissingStickyFault && !shooterMotor2.isConnected()) {
      RobotPreferences.recordStickyFaults("Shooter-Motor2");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Shooter Motor 2 Missing");
      recordedShooterMotor2MissingStickyFault = true;
    }

    if(!recordedHoodMotor1MissingStickyFault && !hoodMotor1.isConnected()) {
      RobotPreferences.recordStickyFaults("Hood-Motor1");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Hood Motor 1 Missing");
      recordedHoodMotor1MissingStickyFault = true;
    }

    if(!recordedHoodMotor2MissingStickyFault && !hoodMotor2.isConnected()) {
      RobotPreferences.recordStickyFaults("Hood-Motor2");
      DataLogUtil.writeMessageEcho("STICKY FAULT Shooter: Hood Motor 2 Missing");
      recordedHoodMotor2MissingStickyFault = true;
    }
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    // Stopping hood motion when at min/max extensions to prevent motor from dying
    if (hood1PositionControlOn) {
      if ((getHoodAnalogVoltage(1) < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01) 
          || (getHoodAnalogVoltage(1) > ShooterConstants.hood1MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

        setHoodVoltageOneMotor(0, 1);
      }
    } else {
      if ((getHoodAnalogVoltage(1) < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hood1VoltageSetpoint < 0) 
          || (getHoodAnalogVoltage(1) > ShooterConstants.hood1MaximumAnalogVoltageCutoff && hood1VoltageSetpoint > 0)) {

        setHoodVoltageOneMotor(0, 1);
      }
    }

    if (hood2PositionControlOn) {
      if ((getHoodAnalogVoltage(2) < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hoodAngleSetpoint <= ShooterConstants.hoodMinimumAngle + 0.01)
          || (getHoodAnalogVoltage(2) > ShooterConstants.hood2MaximumAnalogVoltageCutoff && hoodAngleSetpoint >= ShooterConstants.hoodMaximumAngle - 0.01)) {

        setHoodVoltageOneMotor(0, 2);
      }
    } else {
      if ((getHoodAnalogVoltage(2) < ShooterConstants.hoodMinimumAnalogVoltageCutoff && hood2VoltageSetpoint < 0) 
          || (getHoodAnalogVoltage(2) > ShooterConstants.hood2MaximumAnalogVoltageCutoff && hood2VoltageSetpoint > 0)) {

        setHoodVoltageOneMotor(0, 2);
      }
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();

      SmartDashboard.putNumber(SHOOTER_LEFT_TEMP.value, shooter1Temperature.refresh().getValueAsDouble());
      SmartDashboard.putNumber(SHOOTER_RIGHT_TEMP.value, shooter2Temperature.refresh().getValueAsDouble());
      SmartDashboard.putNumber(SHOOTER_RPM.value, getShooterVelocity());
      SmartDashboard.putNumber(SHOOTER_TARGET_RPM.value, shooterSetpointRPM);
      
      SmartDashboard.putNumber(HOOD_LEFT_VOLTAGE.value, hoodMotor1.getMotorVoltage(true).getValueAsDouble());
      SmartDashboard.putNumber(HOOD_RIGHT_VOLTAGE.value, hoodMotor2.getMotorVoltage(true).getValueAsDouble());
      SmartDashboard.putNumber(HOOD_LEFT_ANALOG_VOLTAGE.value, getHoodAnalogVoltage(1));
      SmartDashboard.putNumber(HOOD_RIGHT_ANALOG_VOLTAGE.value, getHoodAnalogVoltage(2));
      SmartDashboard.putNumber(HOOD_LEFT_LENGTH.value, getHoodExtensionLength(1));
      SmartDashboard.putNumber(HOOD_RIGHT_LENGTH.value, getHoodExtensionLength(2));
      SmartDashboard.putNumber(HOOD_LEFT_ANGLE.value, getHoodAngle(1));
      SmartDashboard.putNumber(HOOD_RIGHT_ANGLE.value, getHoodAngle(2));
      SmartDashboard.putNumber(HOOD_ANGLE.value, getHoodAngle());
      SmartDashboard.putNumber(HOOD_ANGLE_CURRENT_SETPOINT.value, hoodAngleSetpoint);
    }
  }
}