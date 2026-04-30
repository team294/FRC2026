// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.Constants.Ports;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberLimit;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ClimberConstants.ClimberSide;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
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
  private final StatusSignal<Angle> climberLeftEncoderPosition;            // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> climberLeftVelocity;         // Motor velocity measured by the encoder, in rotations/second

  // Status signals for the left climber motor
  private final StatusSignal<Temperature> climberRightTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> climberRightStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Voltage> climberRightVoltage;                  // Motor voltage, in volts
  private final StatusSignal<Angle> climberRightEncoderPosition;            // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> climberRightVelocity;         // Motor velocity measured by the encoder, in rotations/second

  private final VoltageOut climberVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage climberMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(false);

  // DataLog entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogClimberLeftTemp = new DoubleLogEntry(log, "/ClimberLeft/Temperature");
  private final DoubleLogEntry dLogClimberLeftCurrent = new DoubleLogEntry(log, "/ClimberLeft/Current");
  private final DoubleLogEntry dLogClimberLeftVoltage = new DoubleLogEntry(log, "/ClimberLeft/Voltage");
  private final DoubleLogEntry dLogClimberLeftPosition = new DoubleLogEntry(log, "/ClimberLeft/Position");
  private final DoubleLogEntry dLogClimberRightTemp = new DoubleLogEntry(log, "/ClimberRight/Temperature");
  private final DoubleLogEntry dLogClimberRightCurrent = new DoubleLogEntry(log, "/ClimberRight/Current");
  private final DoubleLogEntry dLogClimberRightVoltage = new DoubleLogEntry(log, "/ClimberRight/Voltage");
  private final DoubleLogEntry dLogClimberRightPosition = new DoubleLogEntry(log, "/ClimberRight/Position");

  private final BooleanLogEntry dLogClimberLeftCalibrated = new BooleanLogEntry(log, "/ClimberLeft/isCalibrated");
  private final BooleanLogEntry dLogClimberRightCalibrated = new BooleanLogEntry(log, "/ClimberRight/isCalibrated");

  private final DoubleLogEntry dLogClimberTargetPosition = new DoubleLogEntry(log, "/Climber/TargetPosition");

  // Current setpoint percent output (logical percent; sign indicates direction)
  private double setpointPercentLeft = 0.0;
  private double setpointPercentRight = 0.0;

  private double setpointPosition = 0.0;
  private boolean isClimberInPositionControl = false;

  private boolean climberCalibrated = false; // Default to climber being uncalibrated. Calibrate when left and right climbers are calibrated.
  private boolean climberLeftCalibrated = false;   // Default to being uncalibrated. Calibrate on boot if at lower limit, or recalibrate if "Climber Calibrate L Zero" command run.
  private boolean climberRightCalibrated = false;  // Default to being uncalibrated. Calibrate on boot if at lower limit, or recalibrate if "Climber Calibrate R Zero" command run.

  private double leftEncoderZero;
  private double rightEncoderZero;

  // Sticky Faults
  private boolean climberLeftMissingStickyFault = false;
  private boolean climberRightMissingStickyFault = false;

  public Climber(String subsystemName) {
    this.subsystemName = subsystemName;
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Get status signals for left climber motor
    climberLeftTemp = climberLeftMotor.getDeviceTemp();
    climberLeftStatorCurrent = climberLeftMotor.getStatorCurrent();
    climberLeftVoltage = climberLeftMotor.getMotorVoltage();
    climberLeftEncoderPosition = climberLeftMotor.getPosition();
    climberLeftVelocity = climberLeftMotor.getVelocity();

    // Get status signals for left climber motor
    climberRightTemp = climberRightMotor.getDeviceTemp();
    climberRightStatorCurrent = climberRightMotor.getStatorCurrent();
    climberRightVoltage = climberRightMotor.getMotorVoltage();
    climberRightEncoderPosition = climberRightMotor.getPosition();
    climberRightVelocity = climberRightMotor.getVelocity();

    // Configure motors independently (they drive opposite physical directions).
    // Logical percent sign is consistent across both sides.
    climberLeftConfig = new TalonFXConfiguration();
    climberLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberLeftConfig.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage;
    climberLeftConfig.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage;
    climberLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    climberLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; // Time from 0 to full power, in seconds
    climberLeftConfig.CurrentLimits.SupplyCurrentLimit = 60.0;      // Upper limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0; // Lower limit for the current, in amps
    climberLeftConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;   // Threshold time, in seconds
    climberLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Turn off soft limits until encoder is calibrated
    climberLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    climberLeftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    climberLeftConfig.Feedback.RotorToSensorRatio = 1.0;
    climberLeftConfig.Feedback.SensorToMechanismRatio = ClimberConstants.climberLeftGearRatio;
    climberLeftConfig.ClosedLoopGeneral.ContinuousWrap = false;

    climberLeftConfig.Slot0.kP = ClimberConstants.kP;
    climberLeftConfig.Slot0.kI = ClimberConstants.kI;
    climberLeftConfig.Slot0.kD = ClimberConstants.kD;
    climberLeftConfig.Slot0.kS = ClimberConstants.kS;
    climberLeftConfig.Slot0.kV = ClimberConstants.kV;
    climberLeftConfig.Slot0.kA = ClimberConstants.kA;

    // Configure Magic Motion settings
    climberLeftConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMCruiseVelocity;
    climberLeftConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMAcceleration;
    climberLeftConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MMJerk;

    climberRightConfig = new TalonFXConfiguration();
    // Requirement: right side should be counterclockwise_positive.
    climberRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberRightConfig.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage;
    climberRightConfig.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage;
    climberRightConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
    climberRightConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    climberRightConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    climberRightConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
    climberRightConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    climberRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Turn off soft limits until encoder is calibrated
    climberRightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberRightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    climberRightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    climberRightConfig.Feedback.RotorToSensorRatio = 1.0;
    climberRightConfig.Feedback.SensorToMechanismRatio = ClimberConstants.climberRightGearRatio;
    climberRightConfig.ClosedLoopGeneral.ContinuousWrap = false;

    climberRightConfig.Slot0.kP = ClimberConstants.kP;
    climberRightConfig.Slot0.kI = ClimberConstants.kI;
    climberRightConfig.Slot0.kD = ClimberConstants.kD;
    climberRightConfig.Slot0.kS = ClimberConstants.kS;
    climberRightConfig.Slot0.kV = ClimberConstants.kV;
    climberRightConfig.Slot0.kA = ClimberConstants.kA;

    // Configure Magic Motion settings
    climberRightConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMCruiseVelocity;
    climberRightConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMAcceleration;
    climberRightConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MMJerk;

    climberMMVoltageControl.Slot = 0;
    climberMMVoltageControl.OverrideBrakeDurNeutral = true;

    // Apply the configurations to the motors.
    // This is a blocking call and will wait up to 200ms to apply.
    climberLeftConfigurator.apply(climberLeftConfig);
    Wait.waitTime(250);
    climberRightConfigurator.apply(climberRightConfig);
    Wait.waitTime(250);

    // Stop the climber motors
    stopClimberMotors();

    // Prime the DataLog to reduce delay when first enabling the robot
    DataLogUtil.writeMessage(subsystemName, ": Constructed");
    updateLog(true);

  }

  /**
   * Sets the percent output of both climber motors using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   */
  public void setClimberPercentOutput(double percent) {
    setClimberPercentOutput(percent, ClimberSide.LEFT);
    setClimberPercentOutput(percent, ClimberSide.RIGHT);
  }

  /**
   * Sets the percent output of one climber motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   * @param side which climber side to control
   */
  public void setClimberPercentOutput(double percent, ClimberSide side) {
    isClimberInPositionControl = false;

    if (climberCalibrated) {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxPercentOutput, ClimberConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxUncalibratedPercentOutput, ClimberConstants.maxUncalibratedPercentOutput);
    }

    if (side == ClimberSide.LEFT) {
      setpointPercentLeft = percent;
      climberLeftMotor.setControl(climberVoltageControl.withOutput(percent * ClimberConstants.compensationVoltage));
    } else {
      setpointPercentRight = percent;
      climberRightMotor.setControl(climberVoltageControl.withOutput(percent * ClimberConstants.compensationVoltage * ClimberConstants.rightVoltageRatio));
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
  public void stopClimberMotor(ClimberSide side) {
    setClimberPercentOutput(0.0, side);
  }

  /**
   * Returns the current side setpoint percent output.
   *
   * @param side which side to query
   * @return percent output being applied to that side, -1.0 to 1.0 (positive = up, negative = down)
   */
  public double getClimberSetpointPercent(ClimberSide side) {
    return side == ClimberSide.LEFT ? setpointPercentLeft : setpointPercentRight;
  }

  /**
   * Set position of both climber motors.  If climber is not calibrated, then does nothing.
   * @param position position, in inches from bottom, to set climber to
   */
  public void setClimberPosition(double position) {
    if (climberCalibrated) {
      setpointPosition = MathUtil.clamp(position, ClimberPosition.LOWER_LIMIT.value, ClimberPosition.UPPER_LIMIT.value);
      isClimberInPositionControl = true;

      climberLeftMotor.setControl(climberMMVoltageControl.withPosition(convertClimberInchesToEncoderRotations(setpointPosition) + leftEncoderZero));

      if (setpointPosition == ClimberPosition.UPPER_LIMIT.value) {
        climberRightMotor.setControl(climberMMVoltageControl.withPosition(convertClimberInchesToEncoderRotations(setpointPosition + 0.5) + rightEncoderZero));  // avoid right side's hook clipping rung
      } else {
        climberRightMotor.setControl(climberMMVoltageControl.withPosition(convertClimberInchesToEncoderRotations(setpointPosition) + rightEncoderZero));
      }
    } else {
      DataLogUtil.writeMessageEcho("Climber.SetClimberPosition:  Uncalibrated, position control aborted!");
    }
  }

  /**
   * Get position (in motor rotations) of climber, averaging left and right
   * @return average position of climber
   */
  public double getClimberPositionRotationsRaw() {
    return (getClimberPositionRotationsRaw(ClimberSide.LEFT) + getClimberPositionRotationsRaw(ClimberSide.RIGHT)) / 2.0;
  }

  /**
   * Get position (in motor rotations) of one side of the climber (left or right)
   * @param side
   * @return position of specified side of climber
   */
  public double getClimberPositionRotationsRaw(ClimberSide side) {
    return side == ClimberSide.LEFT
      ? climberLeftEncoderPosition.refresh().getValueAsDouble() - leftEncoderZero
      : climberRightEncoderPosition.refresh().getValueAsDouble() - rightEncoderZero;
  }

  /**
   * Get position (in inches) of climber, averaging left and right
   * @return average position of climber, in inches from bottom
   */
  public double getClimberPositionInches() {
    return (getClimberPositionInches(ClimberSide.LEFT) + getClimberPositionInches(ClimberSide.RIGHT)) / 2.0;
  }

  /**
   * Get position (in inches) of one side of the climber (left or right)
   * @param side
   * @return position of specified side of climber, in inches from bottom
   */
  public double getClimberPositionInches(ClimberSide side) {
    return getClimberPositionRotationsRaw(side) * ClimberConstants.inchesPerEncoderRotation;
  }

  /**
   * Convert inches into climber encoder rotations
   * @param inches length (inches)
   * @return encoder rotations (fully lowered climber = 0 rotations)
   */
  private double convertClimberInchesToEncoderRotations(double inches) {
    return inches / ClimberConstants.inchesPerEncoderRotation;
  }

  /**
   * Get current climber position setpoint
   * @return climber position setpoint, in inches from bottom.  -999.0 if climber is in percent output control mode.
   */
  public double getClimberTargetPosition() {
    return isClimberInPositionControl ? setpointPosition : -999.0;
  }

  /**
   * Get whether or not climber is currently in position control
   * @return true = in position control, false = in percent output control
   */
  public boolean isClimberInPositionControl() {
    return isClimberInPositionControl;
  }

  /**
   * Gets the temperature of the Climber motor
   * @param side motor to report
   * @return climber temperature, in degrees celsius
   */
  public double getClimberTemp(ClimberSide side) {
    return side == ClimberSide.LEFT ? climberLeftTemp.refresh().getValueAsDouble() : climberRightTemp.refresh().getValueAsDouble();
  }

  /**
   * Gets the stator current of the Climber motor.
   * @param side motor to report
   * @return motor current, in amps
   */
  public double getClimberAmps(ClimberSide side) {
    return side == ClimberSide.LEFT ? climberLeftStatorCurrent.refresh().getValueAsDouble() : climberRightStatorCurrent.refresh().getValueAsDouble();
  }

  /**
   * Reads the voltage applied to the Climber motor.
   * @param side motor to report
   * @return climber voltage, in volts
   */
  public double getClimberVoltage(ClimberSide side) {
    return side == ClimberSide.LEFT ? climberLeftVoltage.refresh().getValueAsDouble() : climberRightVoltage.refresh().getValueAsDouble();
  }

 /**
   * Reads the velocity of the Climber motor.
   * @param side motor to report
   * @return climber velocity, in inches/sec
   */
  public double getClimberVelocity(ClimberSide side) {
    return ClimberConstants.inchesPerEncoderRotation * (side == ClimberSide.LEFT ? climberLeftVelocity.refresh().getValueAsDouble() : climberRightVelocity.refresh().getValueAsDouble());
  }

  /**
   * Returns if the climber is in trench configuration
   * Sometimes only one limit triggers when all the way down, 
   * so we just check if either limit is triggered
   * @return true = climber is all the way down, false = climber isn't all the way down.
   */
  public boolean isSafeForTrench() {
    return isSideLimitReached(ClimberSide.LEFT, ClimberLimit.LOWER) || isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.LOWER);
  }

  /**
   * Checks if the provided side is physically at a limit.
   *
   * @param side which side to read (LEFT or RIGHT)
   * @param limit which limit switch to read (LOWER or UPPER)
   * @return true if the side is physically at the selected limit
   */
  public boolean isSideLimitReached(ClimberSide side, ClimberLimit limit) {
    if (side == ClimberSide.LEFT) return limit == ClimberLimit.LOWER ? !lowerLimitLeft.get() : !upperLimitLeft.get();
    return limit == ClimberLimit.LOWER ? !lowerLimitRight.get() : !upperLimitRight.get();
  }

  /**
   * Checks whether the provided setpoint would be blocked by the given side's limits.
   *
   * Convention:
   * - setpointPercent < 0.0 means moving toward the lower limit
   * - setpointPercent > 0.0 means moving toward the upper limit
   */
  public boolean isSideLimitHitForSetpoint(ClimberSide side, double setpointPercent) {
    if (setpointPercent < 0.0) return isSideLimitReached(side, ClimberLimit.LOWER);
    if (setpointPercent > 0.0) return isSideLimitReached(side, ClimberLimit.UPPER);
    return false;
  }

  /**
   * Checks whether either side would be blocked by the provided setpoint direction.
   */
  public boolean isEitherSideLimitHitForSetpoint(double setpointPercent) {
    return isSideLimitHitForSetpoint(ClimberSide.LEFT, setpointPercent)
        || isSideLimitHitForSetpoint(ClimberSide.RIGHT, setpointPercent);
  }

  // ************ Encoder methods

  /**
   * Assumes left climber is at its lower limit. Zeroes the
   * encoder and waits up to 200ms for the new setting to be applied.
   */
  private void zeroEncoderLeft() {
    stopClimberMotor(ClimberSide.LEFT);

    leftEncoderZero = 0.0;
    leftEncoderZero = getClimberPositionRotationsRaw(ClimberSide.LEFT);
    // climberLeftMotor.setPosition(ClimberPosition.LOWER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation);

    DataLogUtil.writeMessageEcho(subsystemName, "Left: ZeroEncoder - success");
  }

  /**
   * Assumes right climber is at its lower limit. Zeroes the
   * encoder and waits up to 200ms for the new setting to be applied.
   */
  private void zeroEncoderRight() {
    stopClimberMotor(ClimberSide.RIGHT);

    rightEncoderZero = 0.0;
    rightEncoderZero = getClimberPositionRotationsRaw(ClimberSide.RIGHT);
    // climberRightMotor.setPosition(ClimberPosition.LOWER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation);

    DataLogUtil.writeMessageEcho(subsystemName, "Right: ZeroEncoder - success");
  }

  /**
   * Assumes left climber is at its lower limit. Zeros the encoder, enables soft limits,
   * sets the climberLeftCalibrated flag, and waits up to 200ms for the new setting to be applied.
   */
  public void calibrateEncoderLeft() {
    zeroEncoderLeft();
    climberLeftCalibrated = true;

    // Set software limits after setting encoderZero
    climberLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberPosition.UPPER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation + leftEncoderZero;
    climberLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberPosition.LOWER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation + leftEncoderZero;
    climberLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // Apply the configurations to left motor
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    climberLeftConfigurator.apply(climberLeftConfig);

    DataLogUtil.writeMessageEcho(subsystemName, ":  checkAndCalibrateEncoderLeft... Calibrated!");
  }

  /**
   * Assumes right climber is at its lower limit. Zeros the encoder, enables soft limits,
   * sets the climberRightCalibrated flag, and waits up to 200ms for the new setting to be applied.
   */
  public void calibrateEncoderRight() {
    zeroEncoderRight();
    climberRightCalibrated = true;

    // Set software limits after setting encoderZero
    climberRightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberPosition.UPPER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation + rightEncoderZero;
    climberRightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberPosition.LOWER_LIMIT.value / ClimberConstants.inchesPerEncoderRotation + rightEncoderZero;
    climberRightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberRightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // Apply the configurations to right motor
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    climberRightConfigurator.apply(climberRightConfig);

    DataLogUtil.writeMessageEcho(subsystemName, ":  checkAndCalibrateEncoderRight... Calibrated!");
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

      dLogClimberLeftTemp.append(getClimberTemp(ClimberSide.LEFT), timeNow);
      dLogClimberLeftCurrent.append(getClimberAmps(ClimberSide.LEFT), timeNow);
      dLogClimberLeftVoltage.append(getClimberVoltage(ClimberSide.LEFT), timeNow);
      dLogClimberLeftPosition.append(getClimberPositionInches(ClimberSide.LEFT));
      dLogClimberRightTemp.append(getClimberTemp(ClimberSide.RIGHT), timeNow);
      dLogClimberRightCurrent.append(getClimberAmps(ClimberSide.RIGHT), timeNow);
      dLogClimberRightVoltage.append(getClimberVoltage(ClimberSide.RIGHT), timeNow);
      dLogClimberRightPosition.append(getClimberPositionInches(ClimberSide.RIGHT));

      dLogClimberLeftCalibrated.append(climberLeftCalibrated);
      dLogClimberRightCalibrated.append(climberRightCalibrated);

      dLogClimberTargetPosition.append(setpointPosition);
    }
  }

  /**
   * Checks for fault conditions and records stick faults if any exist.
   */
  private void checkForFaultConditions() {
      // Resest sticky fault recording if the Sticky Faults were cleared
      climberLeftMissingStickyFault = climberLeftMissingStickyFault && RobotPreferences.isStickyFaultActive();
      climberRightMissingStickyFault = climberRightMissingStickyFault && RobotPreferences.isStickyFaultActive();

      // If we haven't recorded the sticky fault yet, then check for fault conditions
      if (!climberLeftMissingStickyFault && !climberLeftMotor.isConnected()) {
        RobotPreferences.recordStickyFaults("Climber-LeftMotor");
        DataLogUtil.writeMessageEcho("STICKY FAULT Climber: Climber Left Motor Missing");
        climberLeftMissingStickyFault = true;
      }

      if (!climberRightMissingStickyFault && !climberRightMotor.isConnected()) {
        RobotPreferences.recordStickyFaults("Climber-RightMotor");
        DataLogUtil.writeMessageEcho("STICKY FAULT Climber: Climber Right Motor Missing");
        climberRightMissingStickyFault = true;
      }
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    // if at lower limit:
    // - calibrate if not already calibrated
    // - if trying to move down, stop moving (percent and position setpoints)
    // if at upper limit:
    // - if trying to move up, stop moving (percent and position setpoints)
    if (isSideLimitReached(ClimberSide.LEFT, ClimberLimit.LOWER)) {
      if (!climberLeftCalibrated) calibrateEncoderLeft();
      if (!isClimberInPositionControl && setpointPercentLeft < 0.0) stopClimberMotor(ClimberSide.LEFT);
      if (isClimberInPositionControl && setpointPosition < getClimberPositionInches(ClimberSide.LEFT)) stopClimberMotor(ClimberSide.LEFT);
    } else if (isSideLimitReached(ClimberSide.LEFT, ClimberLimit.UPPER)) {
      if (!isClimberInPositionControl && setpointPercentLeft > 0.0) stopClimberMotor(ClimberSide.LEFT);
      if (isClimberInPositionControl && setpointPosition > getClimberPositionInches(ClimberSide.LEFT)) stopClimberMotor(ClimberSide.LEFT);
    }

    // if at lower limit:
    // - calibrate if not already calibrated
    // - if trying to move down, stop moving (percent and position setpoints)
    // if at upper limit:
    // - if trying to move up, stop moving (percent and position setpoints)
    if (isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.LOWER)) {
      if (!climberRightCalibrated) calibrateEncoderRight();
      if (!isClimberInPositionControl && setpointPercentRight < 0.0) stopClimberMotor(ClimberSide.RIGHT);
      if (isClimberInPositionControl && setpointPosition < getClimberPositionInches(ClimberSide.RIGHT)) stopClimberMotor(ClimberSide.RIGHT);
    } else if (isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.UPPER)) {
      if (!isClimberInPositionControl && setpointPercentRight > 0.0) stopClimberMotor(ClimberSide.RIGHT);
      if (isClimberInPositionControl && setpointPosition > getClimberPositionInches(ClimberSide.RIGHT)) stopClimberMotor(ClimberSide.RIGHT);
    }

    // once both sides are calibrated, mark climber as calibrated
    if (!climberCalibrated && (climberLeftCalibrated && climberRightCalibrated)) {
      climberCalibrated = true;
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      checkForFaultConditions();

      SmartDashboard.putBoolean("Climber L LOWER", isSideLimitReached(ClimberSide.LEFT, ClimberLimit.LOWER));
      SmartDashboard.putBoolean("Climber R LOWER", isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.LOWER));
      SmartDashboard.putBoolean("Climber L UPPER", isSideLimitReached(ClimberSide.LEFT, ClimberLimit.UPPER));
      SmartDashboard.putBoolean("Climber R UPPER", isSideLimitReached(ClimberSide.RIGHT, ClimberLimit.UPPER));
      SmartDashboard.putBoolean("Climber Calibrated?", climberCalibrated);
      SmartDashboard.putBoolean("Climber L Calibrated?", climberLeftCalibrated);
      SmartDashboard.putBoolean("Climber R Calibrated?", climberRightCalibrated);
      SmartDashboard.putNumber("Climber L Temp", getClimberTemp(ClimberSide.LEFT));
      SmartDashboard.putNumber("Climber R Temp", getClimberTemp(ClimberSide.RIGHT));
      SmartDashboard.putNumber("Climber L Raw Rotations", getClimberPositionRotationsRaw(ClimberSide.LEFT));
      SmartDashboard.putNumber("Climber R Raw Rotations", getClimberPositionRotationsRaw(ClimberSide.RIGHT));
      SmartDashboard.putNumber("Climber L Position", getClimberPositionInches(ClimberSide.LEFT));
      SmartDashboard.putNumber("Climber R Position", getClimberPositionInches(ClimberSide.RIGHT));
      SmartDashboard.putNumber("Climber Position", getClimberPositionInches());
    }
  }
}
