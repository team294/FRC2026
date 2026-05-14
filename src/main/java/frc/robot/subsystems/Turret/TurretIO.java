// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    // Connection status
    public boolean motorConnected = false;
    public boolean beltEncoderConnected = false;
    public boolean gearEncoderConnected = false;

    // Calibration / control mode state
    public boolean isCalibrated = false;
    public boolean usingBeltEncoder = false;
    public boolean inPositionControl = false;

    // Turret position and velocity (degrees; valid only when isCalibrated = true)
    public double turretPositionDeg = 0.0;
    public double turretVelocityDegPerSec = 0.0;

    // Raw absolute encoder readings (degrees, [0, 360)), after offset subtraction.
    // These are always valid regardless of calibration state and are used by
    // Turret.java for logging the calibration estimation values.
    public double beltEncoderAbsPositionDeg = 0.0;
    public double gearEncoderAbsPositionDeg = 0.0;

    // Raw continuous (multi-turn) encoder readings, in rotations, after offset subtraction.
    public double beltEncoderRawRotations = 0.0;
    public double gearEncoderRawRotations = 0.0;

    // Motor telemetry
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyVoltage = 0.0;
    public double dutyCycle = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the logged inputs. Must be called every periodic loop. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Runs the turret motor at a specified voltage (open-loop).
   * @param volts output voltage, nominally -12 to +12
   */
  public default void setVoltage(double volts) {}

  /**
   * Commands the turret to a target angle using MotionMagic closed-loop control.
   * Has no effect if the turret is not calibrated.
   * @param degrees target turret angle, in degrees
   *                (0 = turret zero, positive = counter-clockwise, negative = clockwise)
   */
  public default void setPosition(double degrees) {}

  /**
   * Calibrates the turret, declaring its current physical position to be {@code currentAngleDeg}.
   * Sets the encoder zero offset, enables software limits, and marks the turret as calibrated.
   * Has no effect if {@code currentAngleDeg} is outside the configured limit range.
   * <p><b>NOTE:</b> includes a blocking CAN config apply call (~50–70 ms).
   * @param currentAngleDeg angle the turret is physically at right now, in degrees
   */
  public default void calibrate(double currentAngleDeg) {}

  /**
   * Marks the turret as uncalibrated, stops the motor, and disables software limits.
   * <p><b>NOTE:</b> includes a blocking CAN config apply call (~50–70 ms).
   */
  public default void setUncalibrated() {}

  /**
   * Attempts to re-calibrate using both CANcoder readings, if both are connected.
   * Switches back to belt-encoder feedback if it had previously fallen back to the rotor encoder.
   * <p><b>NOTE:</b> may include blocking CAN calls (~250 ms) if encoder mode changes.
   */
  public default void recalibrateAutomatic() {}
}