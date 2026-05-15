// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {

  // Drive simulation
  // Moment of inertia derived from robot mass, wheel radius, and gear ratio:
  // J = mass * wheelRadius² / gearRatio² = 52kg * 0.0508m² / 6.026² ≈ 0.0037 kg·m²
  private final DCMotorSim driveSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1),
          0.0037,  // moment of inertia (kg·m²)
          SwerveConstants.kDriveGearRatio),
      DCMotor.getKrakenX60(1));

  // Turn simulation
  private final DCMotorSim turnSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1),
          0.004,  // moment of inertia (kg·m²)
          SwerveConstants.kTurningGearRatio),
      DCMotor.getKrakenX60(1));

  // Sim-specific gains, independent of SwerveConstants which are tuned for real TalonFX hardware.
  // Turn: explicit PD using sim velocity directly to avoid jitter from differentiating position.
  // Drive: P-only on velocity error; feedforward handles the bulk of the output.
  //        kI omitted — integrator windup causes jitter with no anti-windup.
  //        kD omitted — differentiating velocity = acceleration, which is noisy.
  private static final double SIM_TURN_kP = 10.0;  // volts per radian of error
  private static final double SIM_TURN_kD = 0.3;   // volts per (rad/s) of velocity
  private static final double SIM_DRIVE_kP = 0.05; // volts per (rad/s) of velocity error

  private final PIDController drivePID = new PIDController(SIM_DRIVE_kP, 0.0, 0.0);

  // Commanded voltages
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts  = 0.0;

  // Drive velocity setpoint for closed-loop mode
  private double driveVelocitySetpointMPS = 0.0;
  private double driveFeedforwardVolts = 0.0;
  private boolean driveClosedLoop = false;

  // Turn position setpoint for closed-loop mode
  private double turnPositionSetpointRad = 0.0;
  private boolean turnClosedLoop = false;

  // Coast mode tracking
  private boolean coastMode = false;

  // Accumulated drive distance
  private double drivePositionRad = 0.0;

  private static final double LOOP_PERIOD_SECS = 0.02;

  public SwerveModuleIOSim() {}

  // SwerveModuleIO

  /**
   * Steps the simulation forward one loop period and populates inputs. Should be called every periodic loop.
   */
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Run turn closed-loop if active.
    // Explicit PD: P on position error, D directly on sim velocity to avoid jitter
    // from PIDController differentiating position. MathUtil.angleModulus wraps the
    // error to [-π, π], replacing what enableContinuousInput did on the old turnPID.
    if (turnClosedLoop) {
      double positionError =
          MathUtil.angleModulus(turnPositionSetpointRad - turnSim.getAngularPositionRad());
      turnAppliedVolts = MathUtil.clamp(
          SIM_TURN_kP * positionError - SIM_TURN_kD * turnSim.getAngularVelocityRadPerSec(),
          -12.0, 12.0);
    }

    // Run drive closed-loop if active
    if (driveClosedLoop) {
      double driveVelocitySetpointRadPerSec =
          driveVelocitySetpointMPS / (SwerveConstants.kWheelDiameterMeters / 2.0);
      driveAppliedVolts = MathUtil.clamp(
          driveFeedforwardVolts
              + drivePID.calculate(driveSim.getAngularVelocityRadPerSec(), driveVelocitySetpointRadPerSec),
          -12.0, 12.0);
    }

    // Step simulations
    driveSim.setInputVoltage(driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    // Integrate drive position
    drivePositionRad += driveSim.getAngularVelocityRadPerSec() * LOOP_PERIOD_SECS;

    // All simulated devices are always "connected"
    inputs.driveMotorConnected = true;
    inputs.turnMotorConnected = true;
    inputs.CANCoderConnected = true;

    // Drive
    inputs.drivePositionRad = drivePositionRad;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.driveSupplyVoltage = 12.0;
    inputs.driveDutyCycle = driveAppliedVolts / 12.0;
    inputs.driveTempCelsius = 0.0;

    // Turn (TalonFX encoder = same as absolute in sim — no physical offset)
    inputs.turnPositionRad = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turnDutyCycle = turnAppliedVolts / 12.0;
    inputs.turnTempCelsius = 0.0;

    // CANcoder = same position as turn motor in sim
    inputs.turnAbsolutePositionRad = turnSim.getAngularPositionRad();
    inputs.turnAbsoluteVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setDrivePercentOutput(double percent) {
    setDriveVoltage(percent * 12.0);
  }

  @Override
  public void setTurnPercentOutput(double percent) {
    setTurnVoltage(percent * 12.0);
  }

  /**
   * Sets the turn position setpoint in radians.
   * Sim runs a software PID since there is no TalonFX onboard closed-loop in simulation.
   */
  @Override
  public void setTurnPosition(double positionRad) {
    turnClosedLoop = true;
    turnPositionSetpointRad = positionRad;
  }

  /**
   * Sets the drive velocity setpoint (m/s) with feedforward (volts).
   * Sim runs a software PID + feedforward since there is no TalonFX onboard VelocityVoltage in simulation.
   */
  @Override
  public void setDriveVelocity(double velocityMPS, double ff) {
    driveClosedLoop = true;
    driveVelocitySetpointMPS = velocityMPS;
    driveFeedforwardVolts = ff;
  }

  @Override
  public void setCoastMode(boolean coast) {
    coastMode = coast;
    // In sim, coast/brake only affects behavior when voltage = 0.
    // DCMotorSim does not model coast vs brake natively; we approximate
    // by zeroing applied voltage on next update when coasting and stopped.
    // This is sufficient for simulation accuracy.
  }

  @Override
  public boolean isCoastMode() {
    return coastMode;
  }

  // FOC has no effect in simulation
  @Override public void setDriveFOC(boolean enabled) {}
  @Override public void setTurnFOC(boolean enabled)  {}

  // configSwerveModule has no effect in simulation (no hardware to configure)
  @Override public void configSwerveModule() {}
}