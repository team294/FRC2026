// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretAngle;

public class TurretIOSim implements TurretIO {

  // Turret motor simulation (Kraken X60, single motor through gear reduction)
  private final DCMotorSim turretSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1),
          0.05,  // moment of inertia (kg·m²) — tune to match real robot feel
          TurretConstants.turretToMotorGearRatio),
      DCMotor.getKrakenX60(1));

  // Position closed-loop (sim approximates MotionMagic with a software PID).
  // These gains are independent of TurretConstants — those are tuned for
  // MotionMagic on a real TalonFX and are not representative of a simulated PID.
  private static final double SIM_kP = 0.8; // volts per degree of error
  private static final double SIM_kI = 0.0; // shouldn't need
  private static final double SIM_kD = 0.01; // volts per degree/sec of velocity
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final PIDController positionPID = new PIDController(SIM_kP, SIM_kI, SIM_kD, LOOP_PERIOD_SECS);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double positionSetpointDeg = 0.0;

  // Calibration state — sim starts calibrated at 0 degrees for simplicity
  private boolean isCalibrated = true;


  public TurretIOSim() {
    // No continuous input: the turret does not wrap around
    positionPID.disableContinuousInput();
  }


  // TurretIO

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run position closed-loop if active
    if (closedLoop && isCalibrated) {
      appliedVolts = MathUtil.clamp(
        SIM_kP * (positionSetpointDeg - Math.toDegrees(turretSim.getAngularPositionRad()))
        - SIM_kD * Math.toDegrees(turretSim.getAngularVelocityRadPerSec()),
        -12.0, 12.0);
    }

    turretSim.setInputVoltage(appliedVolts);
    turretSim.update(positionPID.getPeriod());

    // Simulated devices are always "connected"
    inputs.motorConnected = true;
    inputs.beltEncoderConnected = true;
    inputs.gearEncoderConnected = true;

    inputs.isCalibrated = isCalibrated;
    inputs.usingBeltEncoder = true;  // sim always uses the "ideal" encoder
    inputs.inPositionControl = closedLoop;

    double positionDeg = Math.toDegrees(turretSim.getAngularPositionRad());
    inputs.turretPositionDeg = positionDeg;
    inputs.turretVelocityDegPerSec = Math.toDegrees(turretSim.getAngularVelocityRadPerSec());

    // Sim encoders report the same value as the turret position — no offset or backlash
    inputs.beltEncoderAbsPositionDeg = MathUtil.inputModulus(positionDeg, 0.0, 360.0);
    inputs.gearEncoderAbsPositionDeg = MathUtil.inputModulus(positionDeg, 0.0, 360.0);

    // Continuous rotations: sim has no multi-turn drift, so just convert position directly
    inputs.beltEncoderRawRotations = positionDeg / 360.0;
    inputs.gearEncoderRawRotations = positionDeg / 360.0;

    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(turretSim.getCurrentDrawAmps());
    inputs.supplyVoltage = 12.0;
    inputs.dutyCycle = appliedVolts / 12.0;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double degrees) {
    if (!isCalibrated) return;
    closedLoop = true;
    positionSetpointDeg = MathUtil.clamp(degrees,
        TurretAngle.LOWER_LIMIT.value, TurretAngle.UPPER_LIMIT.value);
  }

  @Override
  public void calibrate(double currentAngleDeg) {
    if (currentAngleDeg < TurretAngle.LOWER_LIMIT.value
        || currentAngleDeg > TurretAngle.UPPER_LIMIT.value) return;
    // In sim there is no encoder zero offset to set — the sim position IS the angle.
    // Just mark calibrated.
    isCalibrated = true;
  }

  @Override
  public void setUncalibrated() {
    closedLoop = false;
    appliedVolts = 0.0;
    isCalibrated = false;
  }

  @Override
  public void recalibrateAutomatic() {
    // In sim both encoders are always connected; calibrate to the current sim position.
    calibrate(Math.toDegrees(turretSim.getAngularPositionRad()));
  }
}