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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElasticKeyConstants.ElasticKey;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.Ports;
import frc.robot.utilities.ClimberProfileGenerator;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

public class Climber extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private final String subsystemName;   // Subsystem name for use in file logging and dashboarrd

  // Create variables for the Climber Kraken motor 1
  private final StatusSignal<Voltage> climberMotor1SupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> climberMotor1Temp;       // Motor temperature, in degrees Celsius
  private final StatusSignal<Double> climberMotor1DutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> climberMotor1StatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> climberMotor1EncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> climberMotor1EncoderVelocity;
  private final StatusSignal<Voltage> climberMotor1Voltage;

  // Create variables for the Climber Kraken motor 2
  private final StatusSignal<Voltage> climberMotor2SupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> climberMotor2Temp;       // Motor temperature, in degrees Celsius
  private final StatusSignal<Double> climberMotor2DutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> climberMotor2StatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> climberMotor2EncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> climberMotor2EncoderVelocity;
  private final StatusSignal<Voltage> climberMotor2Voltage;

  private final TalonFX climberMotor1 = new TalonFX(Ports.CANClimberMotor1);
  private final TalonFX climberMotor2 = new TalonFX(Ports.CANClimberMotor2);
  private final TalonFXConfigurator climberMotor1Configurator = climberMotor1.getConfigurator();
  private final TalonFXConfigurator climberMotor2Configurator = climberMotor2.getConfigurator();
  private TalonFXConfiguration climberMotor1Config;
  private TalonFXConfiguration climberMotor2Config;
  private VoltageOut climberMotor1VoltageControl = new VoltageOut(0.0);
  private VoltageOut climberMotor2VoltageControl = new VoltageOut(0.0);

  // Create lower limit and upper limit sensors
  private final DigitalInput lowerLimitSensor1 = new DigitalInput(Ports.DIOClimberLowerLimitSensor1);
  private final DigitalInput lowerLimitSensor2 = new DigitalInput(Ports.DIOClimberLowerLimitSensor2);

  // Create the motion profile generator
  private final ClimberProfileGenerator climberProfile;

  private boolean climberCalibrated = false;       // true = encoder is working and calibrated, false = not calibrated
  private boolean climberPositionControl = false;  // true = position control mode (motion profile), false = manual control mode (percent output)

  // Sticky faults
  private boolean climberMotor1Missing;
  private boolean climberMotor2Missing;

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTemp1 = new DoubleLogEntry(log, "/Climber/Temp1");
  private final DoubleLogEntry dLogPctOut1 = new DoubleLogEntry(log, "/Climber/PctOut1");
  private final DoubleLogEntry dLogCurrent1 = new DoubleLogEntry(log, "/Climber/Current1");
  private final DoubleLogEntry dLogBusVolt1 = new DoubleLogEntry(log, "/Climber/BusVolt1");
  private final DoubleLogEntry dLogEncInch1 = new DoubleLogEntry(log, "/Climber/EncInch1");
  private final DoubleLogEntry dLogVel1 = new DoubleLogEntry(log, "/Climber/Velocity1");
  private final DoubleLogEntry dLogTemp2 = new DoubleLogEntry(log, "/Climber/Temp2");
  private final DoubleLogEntry dLogPctOut2 = new DoubleLogEntry(log, "/Climber/PctOut2");
  private final DoubleLogEntry dLogCurrent2 = new DoubleLogEntry(log, "/Climber/Current2");
  private final DoubleLogEntry dLogBusVolt2 = new DoubleLogEntry(log, "/Climber/BusVolt2");
  private final DoubleLogEntry dLogEncInch2 = new DoubleLogEntry(log, "/Climber/EncInch2");
  private final DoubleLogEntry dLogVel2 = new DoubleLogEntry(log, "/Climber/Velocity2");
  private final DoubleLogEntry dLogTargetPos = new DoubleLogEntry(log, "/Climber/TargetPos");
  private final BooleanLogEntry dLogCalibrated = new BooleanLogEntry(log, "/Climber/Calibrated");
  private final BooleanLogEntry dLogPosControl = new BooleanLogEntry(log, "/Climber/PositionControl");
  private final BooleanLogEntry dLogLowerLimit = new BooleanLogEntry(log, "/Climber/AtLowerLimit");


  /**
   * Creates the climber subsystem.
   * @param subsystemName name of the subsystem for file logging
   */
  public Climber(String subsystemName) {
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects for motor 1
    climberMotor1SupplyVoltage = climberMotor1.getSupplyVoltage();
    climberMotor1Temp = climberMotor1.getDeviceTemp();
    climberMotor1DutyCycle = climberMotor1.getDutyCycle();
    climberMotor1StatorCurrent = climberMotor1.getStatorCurrent();
    climberMotor1EncoderPosition = climberMotor1.getPosition();
    climberMotor1EncoderVelocity = climberMotor1.getVelocity();
    climberMotor1Voltage = climberMotor1.getMotorVoltage();

    // Get signal and sensor objects for motor 2
    climberMotor2SupplyVoltage = climberMotor2.getSupplyVoltage();
    climberMotor2Temp = climberMotor2.getDeviceTemp();
    climberMotor2DutyCycle = climberMotor2.getDutyCycle();
    climberMotor2StatorCurrent = climberMotor2.getStatorCurrent();
    climberMotor2EncoderPosition = climberMotor2.getPosition();
    climberMotor2EncoderVelocity = climberMotor2.getVelocity();
    climberMotor2Voltage = climberMotor2.getMotorVoltage();

    // Instantiate the motion profile generator
    climberProfile = new ClimberProfileGenerator(this);

    // Configure the climber motor 1
    climberMotor1Config = new TalonFXConfiguration();
    climberMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;    // TODO CALIBRATE
    climberMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor1Config.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage * ClimberConstants.maxPercentOutput;
    climberMotor1Config.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage * ClimberConstants.maxPercentOutput;
    climberMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0; // Time from 0 to full power, in seconds
    climberMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    climberMotor1Config.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    climberMotor1Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    climberMotor1Config.CurrentLimits.SupplyCurrentLowerTime = 1.0;    // Threshold time, in seconds
    climberMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to motor 1
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    climberMotor1Configurator.apply(climberMotor1Config);

    // Configure the climber motor 2
    climberMotor2Config = new TalonFXConfiguration();
    climberMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;     // TODO CALIBRATE
    climberMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor2Config.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage;
    climberMotor2Config.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage;
    climberMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0; // Time from 0 to full power, in seconds
    climberMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    climberMotor2Config.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    climberMotor2Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    climberMotor2Config.CurrentLimits.SupplyCurrentLowerTime = 1.0;    // Threshold time, in seconds
    climberMotor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to motor 2
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    climberMotor2Configurator.apply(climberMotor2Config);

    // This is a blocking call and will wait up to 200ms for the zero to apply
    checkAndCalibrateEncoders();

    // Wait 0.25 seconds before checking the limit switch or encoder ticks.
    // The reason is that zeroing the encoder (above) can be delayed up to 50ms for a round trip 
    // from the Rio to the Kraken and back to the Rio. So, reading position could give the wrong value 
    // if we do not wait (random weird behavior). Verified this is still needed after migrating to Phoenix 6.
    // DO NOT GET RID OF THIS WITHOUT TALKING TO DON.
    Wait.waitTime(250);

    // Start the climber in uncalibrated mode unless it is properly zeroed
    climberCalibrated = isClimberAtLowerLimit() && (getClimberEncoderRotations(1) == 0 && getClimberEncoderRotations(2) == 0);

    // Ensure the climber starts in manual control mode
    stopClimberMotors();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
  }

  /**
   * Checks if climber is in position control.
   * @return true = position control mode (motion profile), false = manual control mode (percent output)
   */
  public boolean isClimberInPositionControl() {
    return climberPositionControl;
  }

  // ************ Climber movement methods

  /**
   * Stops both climber motors.
   */
  public void stopClimberMotors() {
    setClimberPercentOutput(0);
  }

  /**
   * Sets the percent output of both climber motors.
   * This method is for internal use only. It does not change manual vs. profile control modes or check interlocks.
   * @param percent output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  private void setClimberPercentOutputDirect(double percent) {
    // If the climber is at or below the lower limit, prevent it from going down any further
    if ((isClimberAtLowerLimit() || (isClimberCalibrated() && getClimberPosition() < ClimberPosition.LOWER_LIMIT.value)) && percent < 0) {
      percent = 0;
    }

    // If the climber is at or above the upper limit, prevent it from going up any further
    if ((isClimberCalibrated() && getClimberPosition() > ClimberPosition.UPPER_LIMIT.value) && percent > 0) {
      percent = 0;
    }

    climberMotor1.setControl(climberMotor1VoltageControl.withOutput(percent * ClimberConstants.compensationVoltage));
    climberMotor2.setControl(climberMotor2VoltageControl.withOutput(percent * ClimberConstants.compensationVoltage));
  }
  
  /**
   * Sets climber to manual control mode with the specified percent output for both motors.
   * Does not move the climber if interlocks should prevent movement.
   * @param percent output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public void setClimberPercentOutput(double percent) {
    climberPositionControl = false;
    climberProfile.disableProfileControl();

    // Clamp speed depending on calibration
    if (climberCalibrated) {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxPercentOutput, ClimberConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxUncalibratedPercentOutput, ClimberConstants.maxUncalibratedPercentOutput);
    }

    setClimberPercentOutputDirect(percent);
  }

  /**
   * Gets the percent output of the voltage compensation limit for an climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double getClimberPercentOutput(int motor) {
    switch (motor) {
      case 1:
        climberMotor1DutyCycle.refresh();
        return climberMotor1DutyCycle.getValueAsDouble();
      case 2:
        climberMotor2DutyCycle.refresh();
        return climberMotor2DutyCycle.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average percent output of the voltage compensation limit for the climber motors.
   * @return output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double getClimberPercentOutput() {
    return (getClimberPercentOutput(1) + getClimberPercentOutput(2)) / 2.0;
  }

  /**
   * Sets the target position for the climber, using motion profile movement.
   * If the climber is not calibrated, the target position will not change.
   * @param position target, in inches (use ClimberConstants.ClimberPosition)
   */
  public void setClimberProfileTarget(double position) {   
    if (isClimberCalibrated()) {
      climberPositionControl = true;
      position = MathUtil.clamp(position, ClimberPosition.LOWER_LIMIT.value, ClimberPosition.UPPER_LIMIT.value);
      climberProfile.setProfileTarget(position);
      DataLogUtil.writeMessage(subsystemName, ": setProfileTarget Allowed, Target =", position);
    } else {
      DataLogUtil.writeMessage(subsystemName, ": setProfileTarget Not-Allowed, Target =", position);
    }
  }

  /**
   * Get the target position that the climber is trying to move to.
   * If the climber is in manual control mode, returns the actual position.
   * If the climber is not calibrated, returns +10 inches from the lower limit.
   * @return target, in inches
   */
  public double getCurrentClimberTarget() {
    if (!isClimberCalibrated()) {
      return ClimberConstants.ClimberPosition.LOWER_LIMIT.value + 10.0;
    } else if (isClimberInPositionControl()) {
      return climberProfile.getFinalPosition();
    } else {
      return getClimberPosition();
    }
  }

  // ************ Encoder methods

  /**
   * Checks if the climber is at the lower limit switch.
   * If it is, then zeros the climber encoder and waits up to 200ms for the new setting to be applied.
   */
  public void checkAndZeroEncoders() {
    if (isClimberAtLowerLimit()) {
      stopClimberMotors(); // Ensure PID loop or motion profile will not move to last set position when resetting encoder

      climberMotor1.setPosition(ClimberPosition.LOWER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick);
      climberMotor2.setPosition(ClimberPosition.LOWER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick);

      DataLogUtil.writeMessageEcho(subsystemName, ":  checkAndZeroEncoders... Zeroed!");
    }
  }

  /**
   * Checks if the climber is at the lower limit switch.
   * If it is, then zeros the climber encoder, enables soft limits, sets the climberCalibrated flag, and waits up to 400ms for the new setting to be applied.
   */
  public void checkAndCalibrateEncoders() {
    if (isClimberAtLowerLimit()) {
      checkAndZeroEncoders();
      climberCalibrated = true;
  
      // Set software limits after setting encoderZero
      climberMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberPosition.UPPER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick;
      climberMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberPosition.LOWER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick;
      climberMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // Apply the configurations to motor 1
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply
      climberMotor1Configurator.apply(climberMotor1Config);

      // Set software limits after setting encoderZero
      climberMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberPosition.UPPER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick;
      climberMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberPosition.LOWER_LIMIT.value / ClimberConstants.kClimberEncoderInchesPerTick;
      climberMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // Apply the configurations to motor 2
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply
      climberMotor2Configurator.apply(climberMotor2Config);

      DataLogUtil.writeMessageEcho(subsystemName, ":  checkAndCalibrateEncoders... Calibrated!");
    }
  }

  /**
   * Checks if both encoders are calibrated.
   * @return true = both calibrated, false = one or both are not calibrated
   */
  public boolean isClimberCalibrated() {
    return climberCalibrated;
  }

  /**
   * Gets the raw encoder rotations of a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return raw encoder reading, in pinion rotations (positive = up, negative = down)
   */
  public double getClimberEncoderRotations(int motor) {
    switch (motor) {
      case 1:
        climberMotor1EncoderPosition.refresh();
        return climberMotor1EncoderPosition.getValueAsDouble();
      case 2:
        climberMotor2EncoderPosition.refresh();
        return climberMotor2EncoderPosition.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the vertical position of a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return position from the bottom of the climber, or the lower limit if uncalibrated, in inches
   */
  public double getClimberPosition(int motor) {
    if (!climberCalibrated) return ClimberPosition.LOWER_LIMIT.value;

    switch (motor) {
      case 1:
        return getClimberEncoderRotations(1) * ClimberConstants.kClimberEncoderInchesPerTick;
      case 2:
        return getClimberEncoderRotations(2) * ClimberConstants.kClimberEncoderInchesPerTick;
      default:
        return ClimberPosition.LOWER_LIMIT.value;
    }
  }

  /**
   * Gets the vertical position of the climber using the average position of the motors.
   * @return position from the bottom of the climber, or the lower limit if uncalibrated, in inches
   */
  public double getClimberPosition() {
    return (getClimberPosition(1) + getClimberPosition(2)) / 2.0;
  }

  /**
   * Gets the velocity of a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return velocity, in in/s (positive = up, negative = down)
   */
  public double getClimberVelocity(int motor) {
    switch (motor) {
      case 1:
        climberMotor1EncoderVelocity.refresh();
        return climberMotor1EncoderVelocity.getValueAsDouble() * ClimberConstants.kClimberEncoderInchesPerTick;
      case 2:
        climberMotor2EncoderVelocity.refresh();
        return climberMotor2EncoderVelocity.getValueAsDouble() * ClimberConstants.kClimberEncoderInchesPerTick;
      default:
        return -9999;
    }
  }

  /**
   * Gets the velocity of the climber using the average of the motors.
   * @return velocity, in in/s (positive = up, negative = down)
   */
  public double getClimberVelocity() {
    return (getClimberVelocity(1) + getClimberVelocity(2)) / 2.0;
  }

  // ************ Sensor methods

  /**
   * Gets whether the climber is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isClimberAtLowerLimit() {
    return !lowerLimitSensor1.get() || !lowerLimitSensor2.get();
  }

  /**
   * Gets whether the climber is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isClimberAtLowerLimit(int sensor) {
    if (sensor == 1) return !lowerLimitSensor1.get();
    else return !lowerLimitSensor2.get();
  }

  // ************ Information methods
  
  /**
   * Gets the temperature of a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return temperature, in degrees Celsius
   */
  public double getMotorTemp(int motor) {
    switch (motor) {
      case 1:
        climberMotor1Temp.refresh();
        return climberMotor1Temp.getValueAsDouble();
      case 2:
        climberMotor2Temp.refresh();
        return climberMotor2Temp.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the voltage for a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return voltage, in volts
   */
  public double getClimberVoltage(int motor) {
    switch (motor) {
      case 1:
        climberMotor1Voltage.refresh();
        return climberMotor1Voltage.getValueAsDouble();
      case 2:
        climberMotor2Voltage.refresh();
        return climberMotor2Voltage.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average voltage of the climber motors.
   * @return voltage, in volts
   */
  public double getClimberVoltage() {
    return (getClimberVoltage(1) + getClimberVoltage(2)) / 2.0;
  }

  /**
   * Gets the current that is being drawn by the windings of a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return current, in amps
   */
  public double getStatorCurrent(int motor) {
    switch (motor) {
      case 1:
        climberMotor1StatorCurrent.refresh();
        return climberMotor1StatorCurrent.getValueAsDouble();
      case 2:
        climberMotor2StatorCurrent.refresh();
        return climberMotor2StatorCurrent.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average current that is being drawn by the windings of the two motors.
   * @return current, in amps
   */
  public double getStatorCurrent() {
    return (getStatorCurrent(1) + getStatorCurrent(2)) / 2.0;
  }

  /**
   * Gets the input bus voltage, measured at the input to the motor controller for a specific climber motor.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return voltage, in volts
   */
  public double getBusVoltage(int motor) {
    switch (motor) {
      case 1:
        climberMotor1SupplyVoltage.refresh();
        return climberMotor1SupplyVoltage.getValueAsDouble();
      case 2:
        climberMotor2SupplyVoltage.refresh();
        return climberMotor2SupplyVoltage.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average input bus voltage, measured at the input to the motor controllers.
   * @param motor 1 = climberMotor1, 2 = climberMotor2
   * @return voltage, in volts
   */
  public double getBusVoltage() {
    return (getBusVoltage(1) + getBusVoltage(2)) / 2.0;
  }

  // ************ Periodic methods

  /**
   * Write information about the subsystem to the log
   * @param logWhenDisabled true = log when enabled or disabled, false = only log when enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();
      dLogTemp1.append(getMotorTemp(1), timeNow);
      dLogPctOut1.append(getClimberPercentOutput(1), timeNow);
      dLogCurrent1.append(getStatorCurrent(1), timeNow);
      dLogBusVolt1.append(getBusVoltage(1), timeNow);
      dLogEncInch1.append(getClimberPosition(1), timeNow);
      dLogVel1.append(getClimberVelocity(1), timeNow);
      dLogTemp2.append(getMotorTemp(2), timeNow);
      dLogPctOut2.append(getClimberPercentOutput(2), timeNow);
      dLogCurrent2.append(getStatorCurrent(2), timeNow);
      dLogBusVolt2.append(getBusVoltage(2), timeNow);
      dLogEncInch2.append(getClimberPosition(2), timeNow);
      dLogVel2.append(getClimberVelocity(2), timeNow);
      dLogTargetPos.append(getCurrentClimberTarget(), timeNow);
      dLogCalibrated.append(climberCalibrated, timeNow);
      dLogPosControl.append(climberPositionControl, timeNow);
      dLogLowerLimit.append(isClimberAtLowerLimit(), timeNow);
    }
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = every cycle, false = every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  private void checkForFaultConditions() {
    // Reset recordedStickyFault booleans if the Sticky Faults were cleared
    climberMotor1Missing = climberMotor1Missing && RobotPreferences.isStickyFaultActive();
    climberMotor2Missing = climberMotor2Missing && RobotPreferences.isStickyFaultActive();

    // If we haven't recorded the sticky fault yet, then check fault condition
    if(!climberMotor1Missing && !climberMotor1.isConnected()) {
      RobotPreferences.recordStickyFaults("Climber");
      DataLogUtil.writeMessageEcho("STICKY FAULT Climber: Motor 1 Missing");
      climberMotor1Missing = true;
    }

    if(!climberMotor2Missing && !climberMotor2.isConnected()) {
      RobotPreferences.recordStickyFaults("Climber");
      DataLogUtil.writeMessageEcho("STICKY FAULT Climber: Motor 2 Missing");
      climberMotor2Missing = true;
    }
  }

  @Override
  public void periodic() {
    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean(ElasticKey.CLIMBER_IS_CALIBRATED.value, isClimberCalibrated());
      SmartDashboard.putBoolean(ElasticKey.CLIMBER_IS_IN_POSITION_CONTROL.value, isClimberInPositionControl());

      SmartDashboard.putNumber(ElasticKey.CLIMBER_POSITION.value, getClimberPosition());
      SmartDashboard.putNumber(ElasticKey.CLIMBER_TARGET.value, getCurrentClimberTarget());
      SmartDashboard.putNumber(ElasticKey.CLIMBER_VELOCITY.value, getClimberVelocity());

      SmartDashboard.putBoolean(ElasticKey.CLIMBER_IS_AT_LOWER_LIMIT.value, isClimberAtLowerLimit());
      SmartDashboard.putBoolean(ElasticKey.CLIMBER_MOTOR_ONE_IS_AT_LOWER_LIMIT.value, isClimberAtLowerLimit(1));
      SmartDashboard.putBoolean(ElasticKey.CLIMBER_MOTOR_TWO_IS_AT_LOWER_LIMIT.value, isClimberAtLowerLimit(2));

      SmartDashboard.putNumber(ElasticKey.CLIMBER_MOTOR_ONE_ROTATIONS.value, getClimberEncoderRotations(1));
      SmartDashboard.putNumber(ElasticKey.CLIMBER_MOTOR_TWO_ROTATIONS.value, getClimberEncoderRotations(2));
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();
      updateLog(false);
    }

    // Sets climber motors to percent output required as determined by motion profile.
    // Only set percent output if the motion profile is enabled.
    // NOTE: If we are using our motion profile control loop, then set the power directly using setClimberMotorPercentOutputDirect().
    // Do not call setClimberMotorPercentOutput(), since that will change climberPositionControl to false (manual control).
    if (isClimberInPositionControl()) {
	    setClimberPercentOutputDirect(climberProfile.trackProfilePeriodic());
    }

    // Climber is not calibrated and is at the lower limit, so update encoder calibration.
    if (!isClimberCalibrated() && isClimberAtLowerLimit()) {
      // This is a blocking call and will wait up to 200ms for the zero to apply.
      checkAndCalibrateEncoders();
      DataLogUtil.writeMessageEcho(subsystemName, ": Calibrate Encoders in periodic(), Climber Pos after cal =", getClimberPosition());
    }

    // Climber is calibrated but the zero position is "drifting upwards."  Re-zero if the climber drifts upwards too much
    if (isClimberCalibrated() && isClimberAtLowerLimit() && 
        getClimberPosition() > 1.0 && getClimberPosition() < 2.0) {
      // This is a blocking call and will wait up to 200ms for the zero to apply.
      double climberPosPre = getClimberPosition();
      checkAndZeroEncoders();
      DataLogUtil.writeMessageEcho(subsystemName, ": Zero Encoders in periodic(), Pos before =", climberPosPre, ", Pos after =", getClimberPosition());
    }

    // If the climber is at or below the lower limit, prevent it from going down any further.
    if (isClimberAtLowerLimit() || (isClimberCalibrated() && getClimberPosition() < ClimberPosition.LOWER_LIMIT.value)) {
      // Climber is calibrated and under position control. If moving down, set the target to the current position.
      if (isClimberCalibrated() && isClimberInPositionControl()) {
        if (getCurrentClimberTarget() < getClimberPosition()) {
          setClimberProfileTarget(getClimberPosition());
        }

      // Climber is under manual control. If moving down, stop the motors.
      } else if (!isClimberInPositionControl()) {
        if (getClimberPercentOutput() < 0) {
          stopClimberMotors();
        }

      // Climber is uncalibrated and under position control. Stop the motors.
      } else {
        stopClimberMotors();
      }
    }

    // If the climber is at or above the upper limit, prevent it from going up any further.
    if (isClimberCalibrated() && getClimberPosition() > ClimberPosition.UPPER_LIMIT.value) {
      // Climber is calibrated and under position control. If moving up, set the target to the current position.
      if (isClimberCalibrated() && isClimberInPositionControl()) {
        if (getCurrentClimberTarget() > getClimberPosition()) {
          setClimberProfileTarget(getClimberPosition());
        }

      // Climber is under direct control. If moving up, stop the motors.
      } else if (!isClimberInPositionControl()) {
        if (getClimberPercentOutput() > 0) {
          stopClimberMotors();
        }
        
      // Climber is uncalibrated and under position control. Stop the motors.
      } else {
        stopClimberMotors();
      }
    }
  }
}
