// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.Ports;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretAngle;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

public class TurretIOTalonFX implements TurretIO {

  // Motor
  private final TalonFX turretMotor = new TalonFX(Ports.CANTurretMotor);
  private final TalonFXConfigurator motorConfigurator = turretMotor.getConfigurator();

  // Two TalonFX configs: one using the remote belt CANcoder, one using the rotor encoder.
  // We switch between them at runtime if the belt encoder disconnects.
  private TalonFXConfiguration remoteEncoderConfig;  // RemoteCANcoder (belt) — preferred
  private TalonFXConfiguration rotorEncoderConfig;   // RotorSensor — fallback

  // Control requests
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(0.0).withEnableFOC(false).withSlot(0).withOverrideBrakeDurNeutral(true);

  // Motor status signals
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<ControlModeValue> controlMode;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorVoltage;

  // Belt CANcoder (primary absolute reference, lower backlash to turret)
  private final CANcoder canCoderBelt = new CANcoder(Ports.CANTurretEncoderBelt);
  private final CANcoderConfigurator beltConfigurator = canCoderBelt.getConfigurator();
  private final StatusSignal<MagnetHealthValue> beltMagnetHealth;
  private final StatusSignal<Angle> beltAbsPosition;
  private final StatusSignal<Angle> beltPosition; // continuous, multi-turn

  // Gear CANcoder (used together with belt encoder to resolve multi-turn ambiguity)
  private final CANcoder canCoderGear = new CANcoder(Ports.CANTurretEncoderGear);
  private final CANcoderConfigurator gearConfigurator = canCoderGear.getConfigurator();
  private final StatusSignal<MagnetHealthValue> gearMagnetHealth;
  private final StatusSignal<Angle> gearAbsPosition;
  private final StatusSignal<Angle> gearPosition; // continuous, multi-turn

  // Calibration state
  private boolean turretCalibrated = false;
  // Zero offset: turretZero = (rawEncoderRotations * turretDegreesPerRotation) - turretAngleDeg
  private double turretZero = 0.0;

  // Which TalonFX feedback config is active
  private boolean usingBeltEncoder = false;

  public TurretIOTalonFX() {
    // Motor signals
    supplyVoltage = turretMotor.getSupplyVoltage();
    temp = turretMotor.getDeviceTemp();
    controlMode = turretMotor.getControlMode();
    dutyCycle = turretMotor.getDutyCycle();
    statorCurrent = turretMotor.getStatorCurrent();
    motorPosition = turretMotor.getPosition();
    motorVelocity = turretMotor.getVelocity();
    motorVoltage = turretMotor.getMotorVoltage();

    // CANcoder signals
    beltMagnetHealth = canCoderBelt.getMagnetHealth();
    beltAbsPosition = canCoderBelt.getAbsolutePosition();
    beltPosition = canCoderBelt.getPosition();
    gearMagnetHealth = canCoderGear.getMagnetHealth();
    gearAbsPosition = canCoderGear.getAbsolutePosition();
    gearPosition = canCoderGear.getPosition();

    // Build and apply CANcoder configs
    CANcoderConfiguration beltConfig = new CANcoderConfiguration();
    beltConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    beltConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    beltConfig.MagnetSensor.MagnetOffset = 0.0;
    beltConfigurator.apply(beltConfig);

    CANcoderConfiguration gearConfig = new CANcoderConfiguration();
    gearConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    gearConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    gearConfig.MagnetSensor.MagnetOffset = 0.0;
    gearConfigurator.apply(gearConfig);

    // Build both TalonFX configs (soft limits disabled until calibrated)
    buildRotorEncoderConfig();
    buildRemoteEncoderConfig();

    // Use belt encoder if it is available — it has less backlash to the turret
    usingBeltEncoder = isBeltEncoderConnected();
    if (usingBeltEncoder) {
      motorConfigurator.apply(remoteEncoderConfig);
      DataLogUtil.writeMessageEcho("TurretIO: Motor using Belt Encoder for motion control.");
    } else {
      motorConfigurator.apply(rotorEncoderConfig);
      DataLogUtil.writeMessageEcho("TurretIO: Belt Encoder not found — using rotor encoder for motion control.");
      RobotPreferences.recordStickyFaults("Turret-Encoders");
    }

    // Give the CANbus time to propagate the new config before reading encoder values
    Wait.waitTime(250);

    // Calibrate automatically if both encoders are healthy
    if (isBeltEncoderConnected() && isGearEncoderConnected()) {
      calibrate(calcAngleFromEncodersAdvanced());
    } else {
      turretCalibrated = false;
      RobotPreferences.recordStickyFaults("Turret-Encoders");
      DataLogUtil.writeMessageEcho("TurretIO: Not calibrated — one or both encoders missing.");
    }

    // Sanity-check limit constants: range must not exceed one full vernier cycle,
    // and lower limit must be ≤ 0 (so the wrap point calculation is valid).
    if (TurretAngle.UPPER_LIMIT.value - TurretAngle.LOWER_LIMIT.value
            > 360.0 * TurretConstants.encoderCalculationCoefficient
        || TurretAngle.LOWER_LIMIT.value > 0.0) {
      turretCalibrated = false;
      RobotPreferences.recordStickyFaults("Turret-Constants");
      DataLogUtil.writeMessageEcho("TurretIO: Not calibrated — TurretConstants limits are invalid.");
    }

    // Start stopped
    turretMotor.setControl(voltageControl.withOutput(0.0));
  }


  // TurretIO

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Refresh all motor signals together for a consistent snapshot
    inputs.motorConnected = BaseStatusSignal.refreshAll(
        supplyVoltage, temp, controlMode, dutyCycle,
        statorCurrent, motorPosition, motorVelocity, motorVoltage).isOK();

    // Refresh CANcoder signals independently (they live on the same CANbus but
    // are separate devices; a motor glitch should not mask encoder health)
    beltMagnetHealth.refresh();
    beltAbsPosition.refresh();
    beltPosition.refresh();
    gearMagnetHealth.refresh();
    gearAbsPosition.refresh();
    gearPosition.refresh();

    inputs.beltEncoderConnected = isBeltEncoderConnected();
    inputs.gearEncoderConnected = isGearEncoderConnected();

    // Detect belt-encoder dropout and fall back to rotor encoder.
    // This is handled here (not in Turret.java) because it requires a hardware config swap.
    if (usingBeltEncoder && !inputs.beltEncoderConnected) {
      usingBeltEncoder = false;
      setUncalibrated(); // disables soft limits, stops motor — blocking ~50-70 ms
      RobotPreferences.recordStickyFaults("Turret-CANcoder");
      DataLogUtil.writeMessageEcho("TurretIO: Belt CANcoder disconnected — falling back to rotor encoder.",
          "Belt abs =", getBeltAbsoluteDeg(),
          ", Gear abs =", getGearAbsoluteDeg(),
          ", Enc raw =", motorPosition.getValueAsDouble(),
          ", Angle =", getTurretEncoderDeg());
    }

    inputs.isCalibrated = turretCalibrated;
    inputs.usingBeltEncoder = usingBeltEncoder;
    inputs.inPositionControl = isInPositionControl();

    // Calibrated turret position and velocity
    inputs.turretPositionDeg = getTurretEncoderDeg();
    inputs.turretVelocityDegPerSec = motorVelocity.getValueAsDouble() * TurretConstants.turretDegreesPerRotation;

    // Raw absolute encoder readings (always valid, used for calibration estimation logging)
    inputs.beltEncoderAbsPositionDeg = getBeltAbsoluteDeg();
    inputs.gearEncoderAbsPositionDeg = getGearAbsoluteDeg();

    // Raw continuous (multi-turn) encoder readings, after offset subtraction
    inputs.beltEncoderRawRotations =
        beltPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoderBelt;
    inputs.gearEncoderRawRotations =
        gearPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoderGear;

    // Motor telemetry
    inputs.appliedVolts = motorVoltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyVoltage = supplyVoltage.getValueAsDouble();
    inputs.dutyCycle = dutyCycle.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double degrees) {
    if (!turretCalibrated) return;
    turretMotor.setControl(positionControl.withPosition(degreesToEncoderRotations(degrees)));
  }

  @Override
  public void calibrate(double currentAngleDeg) {
    // Reject angles outside of the configured travel limits
    if (currentAngleDeg < TurretAngle.LOWER_LIMIT.value
        || currentAngleDeg > TurretAngle.UPPER_LIMIT.value) {
      DataLogUtil.writeMessageEcho("TurretIO: calibrate() rejected — angle out of limits:",
          currentAngleDeg,
          ", Belt abs =", getBeltAbsoluteDeg(),
          ", Gear abs =", getGearAbsoluteDeg());
      return;
    }

    // Stop the motor so it does not jump when the setpoint is recomputed
    turretMotor.setControl(voltageControl.withOutput(0.0));

    turretZero = motorPosition.getValueAsDouble() * TurretConstants.turretDegreesPerRotation - currentAngleDeg;
    turretCalibrated = true;

    DataLogUtil.writeMessageEcho("TurretIO: calibrated.",
        "usingBeltEncoder =", usingBeltEncoder,
        ", Belt abs =", getBeltAbsoluteDeg(),
        ", Gear abs =", getGearAbsoluteDeg(),
        ", turretZero =", turretZero,
        ", enc raw =", motorPosition.getValueAsDouble(),
        ", angle =", getTurretEncoderDeg());

    // Enable software limits now that we know the zero
    TalonFXConfiguration cfg = usingBeltEncoder ? remoteEncoderConfig : rotorEncoderConfig;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        degreesToEncoderRotations(TurretAngle.UPPER_LIMIT.value);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        degreesToEncoderRotations(TurretAngle.LOWER_LIMIT.value);
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfigurator.apply(cfg); // blocking ~50-70 ms
  }

  @Override
  public void setUncalibrated() {
    turretMotor.setControl(voltageControl.withOutput(0.0));
    turretCalibrated = false;

    // Disable soft limits so the turret can be manually driven to recalibrate
    remoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    remoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    rotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable  = false;
    rotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable  = false;

    TalonFXConfiguration cfg = usingBeltEncoder ? remoteEncoderConfig : rotorEncoderConfig;
    motorConfigurator.apply(cfg); // blocking ~50-70 ms

    DataLogUtil.writeMessageEcho("TurretIO: setUncalibrated.",
        "Enc raw =", motorPosition.getValueAsDouble(),
        ", angle =", getTurretEncoderDeg());
  }

  @Override
  public void recalibrateAutomatic() {
    if (!isBeltEncoderConnected() || !isGearEncoderConnected()) {
      DataLogUtil.writeMessage("TurretIO: recalibrateAutomatic failed —",
          "belt connected =", isBeltEncoderConnected(),
          ", gear connected =", isGearEncoderConnected());
      return;
    }

    // Switch back to belt encoder feedback if we had previously fallen back to rotor
    if (!usingBeltEncoder) {
      usingBeltEncoder = true;
      motorConfigurator.apply(remoteEncoderConfig); // blocking ~50-70 ms
      // Give the CANbus time to propagate the new config before reading position
      Wait.waitTime(250);
    }

    calibrate(calcAngleFromEncodersAdvanced());
  }


  // Internal helpers

  /** Returns true if the belt CANcoder is connected and its magnet is readable. */
  private boolean isBeltEncoderConnected() {
    MagnetHealthValue h = beltMagnetHealth.getValue();
    return canCoderBelt.isConnected()
        && (h == MagnetHealthValue.Magnet_Green || h == MagnetHealthValue.Magnet_Orange);
  }

  /** Returns true if the gear CANcoder is connected and its magnet is readable. */
  private boolean isGearEncoderConnected() {
    MagnetHealthValue h = gearMagnetHealth.getValue();
    return canCoderGear.isConnected()
        && (h == MagnetHealthValue.Magnet_Green || h == MagnetHealthValue.Magnet_Orange);
  }

  /**
   * Returns true if the motor is currently running a position-control mode
   * (MotionMagicVoltage, PositionVoltage, or their FOC variants).
   */
  private boolean isInPositionControl() {
    ControlModeValue mode = controlMode.getValue(); // already refreshed in updateInputs()
    return mode == ControlModeValue.MotionMagicVoltage
        || mode == ControlModeValue.PositionVoltage
        || mode == ControlModeValue.MotionMagicVoltageFOC
        || mode == ControlModeValue.PositionVoltageFOC;
  }

  /**
   * Returns the calibrated turret angle from the encoder currently configured on the motor.
   * Returns 0 if not calibrated (caller should check inputs.isCalibrated before using).
   */
  private double getTurretEncoderDeg() {
    return motorPosition.getValueAsDouble() * TurretConstants.turretDegreesPerRotation - turretZero;
  }

  /**
   * Converts a calibrated turret angle (degrees) to the encoder rotation count
   * expected by the TalonFX position/MotionMagic control requests.
   */
  private double degreesToEncoderRotations(double degrees) {
    return (degrees + turretZero) / TurretConstants.turretDegreesPerRotation;
  }

  /**
   * Returns the belt encoder absolute position in degrees [0, 360),
   * after subtracting the configured offset.
   */
  private double getBeltAbsoluteDeg() {
    double rot = beltAbsPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoderBelt;
    if (rot < 0) rot += 1.0;
    return rot * 360.0;
  }

  /**
   * Returns the gear encoder absolute position in degrees [0, 360),
   * after subtracting the configured offset.
   */
  private double getGearAbsoluteDeg() {
    double rot = gearAbsPosition.getValueAsDouble() - TurretConstants.offsetAngleCANcoderGear;
    if (rot < 0) rot += 1.0;
    return rot * 360.0;
  }

  /**
   * Estimates the turret angle using the vernier-scale relationship between
   * both encoder absolute readings. Used for initial calibration only.
   * <p>Output covers a range of {@code 360 * encoderCalculationCoefficient} degrees,
   * centered on the configured travel limits.
   */
  private double calcAngleFromEncodersBasic() {
    double enc1 = getBeltAbsoluteDeg();
    double enc2 = getGearAbsoluteDeg();

    double angle = (enc2 - enc1) * TurretConstants.encoderCalculationCoefficient;
    // Fix wrap-around: enc2 completes a full revolution before enc1 at the top of range
    if (enc2 < enc1) {
      angle += 360.0 * TurretConstants.encoderCalculationCoefficient;
    }

    // Shift from [0, maxAngle) into the range centered on (LOWER_LIMIT, UPPER_LIMIT)
    if (angle > TurretConstants.encoderCalculationWrapPoint) {
      angle -= 360.0 * TurretConstants.encoderCalculationCoefficient;
    }

    return angle;
  }

  /**
   * Higher-accuracy turret angle estimation using the belt encoder absolute reading,
   * corrected for multi-turn ambiguity via {@link #calcAngleFromEncodersBasic()}.
   * Used for initial calibration only.
   */
  private double calcAngleFromEncodersAdvanced() {
    double uncorrected = getBeltAbsoluteDeg() * TurretConstants.gearEncoderToTurretRatio;
    return uncorrected
        + 360.0 * TurretConstants.gearEncoderToTurretRatio
            * (double) Math.round(
                (calcAngleFromEncodersBasic() - uncorrected)
                    / 360.0 / TurretConstants.gearEncoderToTurretRatio);
  }


  // Config builders

  /** Builds {@link #rotorEncoderConfig} (TalonFX internal rotor sensor). */
  private void buildRotorEncoderConfig() {
    rotorEncoderConfig = new TalonFXConfiguration();
    rotorEncoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rotorEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rotorEncoderConfig.Voltage.PeakForwardVoltage = TurretConstants.compensationVoltage;
    rotorEncoderConfig.Voltage.PeakReverseVoltage = -TurretConstants.compensationVoltage;
    rotorEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    rotorEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    rotorEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    rotorEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
    rotorEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    rotorEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rotorEncoderConfig.Slot0.kP = TurretConstants.kP;
    rotorEncoderConfig.Slot0.kI = TurretConstants.kI;
    rotorEncoderConfig.Slot0.kD = TurretConstants.kD;
    rotorEncoderConfig.Slot0.kS = TurretConstants.kS;
    rotorEncoderConfig.Slot0.kV = TurretConstants.kV;
    rotorEncoderConfig.Slot0.kA = TurretConstants.kA;

    rotorEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MMCruiseVelocity;
    rotorEncoderConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MMAcceleration;
    rotorEncoderConfig.MotionMagic.MotionMagicJerk = TurretConstants.MMJerk;

    rotorEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    rotorEncoderConfig.Feedback.RotorToSensorRatio = 1.0;
    rotorEncoderConfig.Feedback.SensorToMechanismRatio = TurretConstants.turretToMotorGearRatio;
    rotorEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Soft limits disabled until calibrated
    rotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    rotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }

  /** Builds {@link #remoteEncoderConfig} (RemoteCANcoder via belt encoder). */
  private void buildRemoteEncoderConfig() {
    remoteEncoderConfig = new TalonFXConfiguration();
    remoteEncoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    remoteEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    remoteEncoderConfig.Voltage.PeakForwardVoltage = TurretConstants.compensationVoltage;
    remoteEncoderConfig.Voltage.PeakReverseVoltage = -TurretConstants.compensationVoltage;
    remoteEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    remoteEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    remoteEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    remoteEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
    remoteEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    remoteEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    remoteEncoderConfig.Slot0.kP = TurretConstants.kP;
    remoteEncoderConfig.Slot0.kI = TurretConstants.kI;
    remoteEncoderConfig.Slot0.kD = TurretConstants.kD;
    remoteEncoderConfig.Slot0.kS = TurretConstants.kS;
    remoteEncoderConfig.Slot0.kV = TurretConstants.kV;
    remoteEncoderConfig.Slot0.kA = TurretConstants.kA;

    remoteEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MMCruiseVelocity;
    remoteEncoderConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.MMAcceleration;
    remoteEncoderConfig.MotionMagic.MotionMagicJerk = TurretConstants.MMJerk;

    remoteEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    remoteEncoderConfig.Feedback.FeedbackRemoteSensorID = Ports.CANTurretEncoderBelt;
    remoteEncoderConfig.Feedback.RotorToSensorRatio = TurretConstants.gearEncoderToTurretRatio * TurretConstants.turretToMotorGearRatio;
    remoteEncoderConfig.Feedback.SensorToMechanismRatio = 1.0 / TurretConstants.gearEncoderToTurretRatio;
    remoteEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Soft limits disabled until calibrated
    remoteEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    remoteEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  }
}