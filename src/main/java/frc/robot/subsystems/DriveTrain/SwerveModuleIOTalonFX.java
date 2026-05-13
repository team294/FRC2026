// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.Wait;

import static frc.robot.utilities.StringUtil.buildString;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private final String name; // "FL", "FR", "BL", "BR" — for logging

  // Drive motor
  private final TalonFX driveMotor;
  private final TalonFXConfigurator driveConfigurator;
  private TalonFXConfiguration driveConfig;
  private VoltageOut driveVoltageControl   = new VoltageOut(0.0).withEnableFOC(false);
  private VelocityVoltage driveVelocityControl = new VelocityVoltage(0.0).withEnableFOC(false);

  // Drive signals
  private final StatusSignal<Voltage> driveSupplyVoltage;
  private final StatusSignal<Temperature> driveTemp;
  private final StatusSignal<Double> driveDutyCycle;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;

  // Turn motor
  private final TalonFX turnMotor;
  private final TalonFXConfigurator turnConfigurator;
  private TalonFXConfiguration turnConfig;
  private VoltageOut turningVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private PositionVoltage turningPositionControl = new PositionVoltage(0.0).withEnableFOC(false);

  // Turn signals
  private final StatusSignal<Temperature> turnTemp;
  private final StatusSignal<Double> turnDutyCycle;
  private final StatusSignal<Current> turnStatorCurrent;
  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;

  // CANcoder
  private final CANcoder cancoder;
  private final CANcoderConfigurator cancoderConfigurator;
  private CANcoderConfiguration cancoderConfig;

  // CANcoder signals
  private final StatusSignal<Angle> cancoderPosition;
  private final StatusSignal<AngularVelocity>  cancoderVelocity;

  // Encoder zeroing
  private double driveEncoderZero = 0.0; // raw pinion rotations at zero
  private double cancoderZero = 0.0; // = -offsetDegrees, in degrees
  private double turningEncoderZero = 0.0; // raw pinion rotations at calibrated zero

  // Constructor args saved for configSwerveModule() re-runs
  private final boolean driveMotorInverted;
  private final boolean turningMotorInverted;
  private final boolean cancoderReversed;
  private final double  turningOffsetDegrees;

  // Coast/brake tracking
  private boolean isCoastMode = false; // boot to brake

  /**
   * Constructs a SwerveModuleIOTalonFX.
   *
   * @param name Module name for logging ("FL", "FR", "BL", "BR")
   * @param driveMotorAddress CANbus address of the drive motor
   * @param turningMotorAddress CANbus address of the turning motor
   * @param cancoderAddress CANbus address of the turning CANcoder
   * @param driveMotorInverted true = invert drive motor direction
   * @param turningMotorInverted true = invert turning motor direction
   * @param cancoderReversed true = CANcoder is reversed
   * @param turningOffsetDegrees Offset degrees for turning encoder zero (absolute magnet position)
   */
  public SwerveModuleIOTalonFX(String name, int driveMotorAddress, int turningMotorAddress, 
                              int cancoderAddress, boolean driveMotorInverted, boolean turningMotorInverted,
                              boolean cancoderReversed, double turningOffsetDegrees) {

    this.name = name;
    this.driveMotorInverted = driveMotorInverted;
    this.turningMotorInverted = turningMotorInverted;
    this.cancoderReversed = cancoderReversed;
    this.turningOffsetDegrees = turningOffsetDegrees;

    // Drive motor
    driveMotor = new TalonFX(driveMotorAddress, Ports.CANbusCANivore);
    driveConfigurator = driveMotor.getConfigurator();

    driveSupplyVoltage = driveMotor.getSupplyVoltage();
    driveTemp = driveMotor.getDeviceTemp();
    driveDutyCycle = driveMotor.getDutyCycle();
    driveStatorCurrent = driveMotor.getStatorCurrent();
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();

    // Turn motor
    turnMotor = new TalonFX(turningMotorAddress, Ports.CANbusCANivore);
    turnConfigurator = turnMotor.getConfigurator();

    turnTemp = turnMotor.getDeviceTemp();
    turnDutyCycle = turnMotor.getDutyCycle();
    turnStatorCurrent = turnMotor.getStatorCurrent();
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();

    // CANcoder
    cancoder = new CANcoder(cancoderAddress, Ports.CANbusCANivore);
    cancoderConfigurator = cancoder.getConfigurator();

    cancoderPosition = cancoder.getPosition();
    cancoderVelocity = cancoder.getVelocity();

    // Build configs and apply
    buildDriveConfig();
    buildTurnConfig();
    buildCancoderConfig();

    configSwerveModule();

    // Boot to brake
    setCoastMode(false);
  }


  // Config builders

  /** Builds driveConfig from SwerveConstants */
  private void buildDriveConfig() {
    driveConfig = new TalonFXConfiguration();

    driveConfig.MotorOutput.Inverted   = driveMotorInverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod   = 0.0;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // Current limits
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // PID Slot 0 for VelocityVoltage
    driveConfig.Slot0.kP = SwerveConstants.driveMotorkP;
    driveConfig.Slot0.kI = SwerveConstants.driveMotorkI;
    driveConfig.Slot0.kD = SwerveConstants.driveMotorkD;

    // VelocityVoltage control request config
    driveVelocityControl.Slot = 0;
    driveVelocityControl.OverrideBrakeDurNeutral = true;
  }

  /** Builds turnConfig from SwerveConstants. */
  private void buildTurnConfig() {
    turnConfig = new TalonFXConfiguration();

    turnConfig.MotorOutput.Inverted   = turningMotorInverted
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turnConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod   = 0.1;
    turnConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Current limits
    turnConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    turnConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    turnConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // PID Slot 0 for PositionVoltage
    turnConfig.Slot0.kP = SwerveConstants.turningMotorkP;
    turnConfig.Slot0.kI = SwerveConstants.turningMotorkI;
    turnConfig.Slot0.kD = SwerveConstants.turningMotorkD;

    // PositionVoltage control request config
    turningPositionControl.Slot = 0;
    turningPositionControl.OverrideBrakeDurNeutral = true;
  }

  /** Builds cancoderConfig. */
  private void buildCancoderConfig() {
    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // signed [-0.5, 0.5)
    cancoderConfig.MagnetSensor.SensorDirection = cancoderReversed
        ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
  }

  // SwerveModuleIO

  @Override
  public void configSwerveModule() {
    // Save NeutralMode state before applying
    isCoastMode = (driveConfig.MotorOutput.NeutralMode == NeutralModeValue.Coast);

    // Apply configs — blocking calls
    driveConfigurator.apply(driveConfig);
    turnConfigurator.apply(turnConfig);
    cancoderConfigurator.apply(cancoderConfig);

    // Give config extra time to apply before reading encoder values
    Wait.waitTime(200);

    // Calibrate encoders
    zeroDriveEncoder();
    calibrateCanCoderDegrees(turningOffsetDegrees);
    calibrateTurningEncoderDegrees(getCanCoderDegrees());
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Refresh all signals together for consistency
    inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
        driveSupplyVoltage, driveTemp, driveDutyCycle,
        driveStatorCurrent, drivePosition, driveVelocity, driveAppliedVolts).isOK();

    inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
        turnTemp, turnDutyCycle, turnStatorCurrent,
        turnPosition, turnVelocity, turnAppliedVolts).isOK();

    inputs.CANCoderConnected = BaseStatusSignal.refreshAll(
        cancoderPosition, cancoderVelocity).isOK();

    // Drive
    inputs.drivePositionRad = getDriveEncoderMeters() / (SwerveConstants.kWheelDiameterMeters / 2.0);
    inputs.driveVelocityRadPerSec = getDriveEncoderVelocity() / (SwerveConstants.kWheelDiameterMeters / 2.0);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyVoltage = driveSupplyVoltage.getValueAsDouble();
    inputs.driveDutyCycle = driveDutyCycle.getValueAsDouble();
    inputs.driveTempCelsius = driveTemp.getValueAsDouble();

    // Turn (TalonFX encoder, un-normalized — may exceed 360 degrees)
    inputs.turnPositionRad = Units.degreesToRadians(getTurningEncoderDegrees());
    inputs.turnVelocityRadPerSec = Units.degreesToRadians(getTurningEncoderVelocityDPS());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnStatorCurrent.getValueAsDouble();
    inputs.turnDutyCycle = turnDutyCycle.getValueAsDouble();
    inputs.turnTempCelsius = turnTemp.getValueAsDouble();

    // Turn (CANcoder — absolute, normalized [-180, 180))
    inputs.turnAbsolutePositionRad = Units.degreesToRadians(getCanCoderDegrees());
    inputs.turnAbsoluteVelocityRadPerSec = Units.degreesToRadians(getCanCoderVelocityDPS());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageControl.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setControl(turningVoltageControl.withOutput(volts));
  }

  @Override
  public void setDrivePercentOutput(double percent) {
    setDriveVoltage(percent * SwerveConstants.voltageCompSaturation);
  }

  @Override
  public void setTurnPercentOutput(double percent) {
    turnMotor.setControl(turningVoltageControl.withOutput(percent * SwerveConstants.voltageCompSaturation));
  }

  @Override
  public void setTurnPosition(double positionRad) {
    // Convert radians to raw turning encoder ticks
    double degrees = Units.radiansToDegrees(positionRad);
    double rawTarget = degrees / SwerveConstants.kTurningEncoderDegreesPerTick + turningEncoderZero;
    turnMotor.setControl(turningPositionControl.withPosition(rawTarget));
  }

  @Override
  public void setDriveVelocity(double velocityMPS, double ff) {
    // Convert m/s to raw drive encoder ticks/sec
    double rawVelocity = velocityMPS / SwerveConstants.kDriveEncoderMetersPerTick;
    driveMotor.setControl(driveVelocityControl.withVelocity(rawVelocity).withFeedForward(ff));
  }

  @Override
  public void setCoastMode(boolean coast) {
    if (coast == isCoastMode) return; // avoid unnecessary CAN traffic
    if (coast) {
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      turnMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      driveMotor.setNeutralMode(NeutralModeValue.Brake);
      turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    isCoastMode = coast;
  }

  @Override
  public boolean isCoastMode() {
    return isCoastMode;
  }

  @Override
  public void setDriveFOC(boolean enabled) {
    driveVelocityControl = driveVelocityControl.withEnableFOC(enabled);
    driveVoltageControl  = driveVoltageControl.withEnableFOC(enabled);
  }

  @Override
  public void setTurnFOC(boolean enabled) {
    turningPositionControl = turningPositionControl.withEnableFOC(enabled);
    turningVoltageControl  = turningVoltageControl.withEnableFOC(enabled);
  }


  // Internal encoder helpers

  // Drive encoder

  /**
   * Gets the encoder position of the drive motor.
   * @return encoder position, in pinion rotations
   */
  private double getDriveEncoderRotations() {
    return drivePosition.getValueAsDouble();
  }

  /**
	 * Sets the drive encoder position to zero in software.
	 */
  private void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRotations();
    DataLogUtil.writeMessageEcho(buildString("SwerveModuleIO ", name),
        ": ZeroDriveEncoder, driveEncoderZero =", driveEncoderZero,
        ", raw encoder =", getDriveEncoderRotations(),
        ", encoder meters =", getDriveEncoderMeters());
  }

  /**
   * Gets the distance traveled by the drive wheel.
   * @return distance traveled, in meters (positive = forward)
   */
  private double getDriveEncoderMeters() {
    return (getDriveEncoderRotations() - driveEncoderZero) * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  /**
   * Gets the velocity of the drive motor.
   * @return velocity, in meters per second (positive = forward)
   */
  private double getDriveEncoderVelocity() {
    return driveVelocity.getValueAsDouble() * SwerveConstants.kDriveEncoderMetersPerTick;
  }

  // Turning TalonFX encoder

  /**
   * Gets the raw encoder position of the turning motor.
   * @return encoder position, in pinion rotations
   */
  private double getTurningEncoderRaw() {
    return turnPosition.getValueAsDouble();
  }

  /**
   * Calibrates the turning motor encoder. Sets the current wheel facing to currentAngleDegrees.
   * @param currentAngleDegrees current angle, in degrees
   */
  private void calibrateTurningEncoderDegrees(double currentAngleDegrees) {
    turningEncoderZero = getTurningEncoderRaw() - (currentAngleDegrees / SwerveConstants.kTurningEncoderDegreesPerTick);
    DataLogUtil.writeMessageEcho(buildString("SwerveModuleIO ", name),
        ": calibrateTurningEncoder, turningEncoderZero =", turningEncoderZero,
        ", raw encoder =", getTurningEncoderRaw(),
        ", set degrees =", currentAngleDegrees,
        ", encoder degrees =", getTurningEncoderDegrees());
  }

  /**
   * Gets the turning motor encoder facing. When calibrated, 0 should be with the wheel pointing toward 
   * the front of robot (positive = counterclockwise, negative = clockwise).
   * @return encoder facing, in degrees (does not wrap, so could be greater than 360 degrees)
   */
  private double getTurningEncoderDegrees() {
    return (getTurningEncoderRaw() - turningEncoderZero) * SwerveConstants.kTurningEncoderDegreesPerTick;
  }

  /**
   * Gest the rotational velocity of the turning encoder.
   * @return turning TalonFX encoder rotational velocity for wheel facing, in degrees/second (positive = counterclockwise, negative = clockwise)
   */
  private double getTurningEncoderVelocityDPS() {
    return turnVelocity.getValueAsDouble() * SwerveConstants.kTurningEncoderDegreesPerTick;
  }

  // CANcoder

  /**
   * Calibrates the CANcoder encoder so that 0 should be with the wheel pointing toward the front of robot.
   * @param offsetDegrees desired encoder zero point, in absolute magnet position reading
   */
  private void calibrateCanCoderDegrees(double offsetDegrees) {
    cancoderZero = -offsetDegrees;
    DataLogUtil.writeMessageEcho(buildString("SwerveModuleIO ", name),
        ": calibrateCanCoder, cancoderZero =", cancoderZero,
        ", raw encoder =", cancoderPosition.getValueAsDouble() * 360.0,
        ", encoder degrees =", getCanCoderDegrees());
  }

  /**
   * Gets the wheel facing of the turning motor. When calibrated, 0 should be with the wheel pointing toward 
   * the front of robot (positive = counterclockwise, negative = clockwise).
   * @return wheel facing, in degrees [-180,+180)
   */
  private double getCanCoderDegrees() {
    // (rawRotations * 360) - cancoderZero, normalized to [-180, 180)
    double raw = MathBCR.normalizeAngle(cancoderPosition.getValueAsDouble() * 360.0 - cancoderZero);
    return raw;
  }

  /**
   * Gets the rotatioal velocity of the turning CANcoder.
   * @return rotational velocity for wheel facing, in degrees/second (positive = counterclockwise, negative = clockwise)
   */
  private double getCanCoderVelocityDPS() {
    return cancoderVelocity.getValueAsDouble() * 360.0;
  }
}