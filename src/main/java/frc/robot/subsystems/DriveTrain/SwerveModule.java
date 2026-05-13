// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.utilities.MathBCR;
import frc.robot.utilities.MathSwerveModuleState;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.StringUtil;
import frc.robot.utilities.DataLogUtil;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  // AdvantageKit
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private final int index;
  private final String name; // "FL", "FR", "BL", "BR"

  // Drive FeedForward
  // Lives here (not in IO) because it is not hardware-specific.
  private final SimpleMotorFeedforward driveFeedforward;

  // State
  private SwerveModuleState setpoint = new SwerveModuleState();
  private boolean isOpenLoop = false;
  private boolean inVelocityMode = true;
  private double priorDriveVelocity = 0.0;

  /**
   * Constructs a SwerveModule
   *
   * @param io    Hardware IO implementation for this module
   * @param index Module index: 0 = FL, 1 = FR, 2 = BL, 3 = BR
   * @param kVm   Drive motor kV multiplier
   *              Drive feedforward kV = kVDriveAvg * kVm.
   */
  public SwerveModule(SwerveModuleIO io, int index, double kVm) {
    this.io = io;
    this.index = index;
    this.name = (index >= 0 && index < MODULE_NAMES.length) ? MODULE_NAMES[index] : String.valueOf(index);

    driveFeedforward = new SimpleMotorFeedforward(
        SwerveConstants.kSDrive,
        SwerveConstants.kVDriveAvg * kVm,
        SwerveConstants.kADrive);
  }

  /**
   * Called every periodic loop by Drive.periodic().
   * Reads and logs all hardware inputs, then re-sends the current setpoints to IO.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Turn: IO runs PositionVoltage on the TalonFX. Re-sending every loop keeps the TalonFX setpoint fresh.
    io.setTurnPosition(setpoint.angle.getRadians());

    // Drive
    if (isOpenLoop) {
      // Open-loop: feedforward voltage only
      double ff = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
      io.setDriveVoltage(ff);
    } else {
      // Closed-loop: kA / velMinDeltaUsingkA
      // priorDriveVelocity is updated in setDesiredState, not here, so this uses the last stored value.
      double ff;
      if (inVelocityMode &&
          Math.abs(priorDriveVelocity - setpoint.speedMetersPerSecond) >= SwerveConstants.velMinDeltaUsingkA) {
        ff = driveFeedforward.calculateWithVelocities(priorDriveVelocity, setpoint.speedMetersPerSecond);
      } else {
        ff = driveFeedforward.calculate(setpoint.speedMetersPerSecond);
      }
      // IO runs VelocityVoltage on the TalonFX with this feedforward
      io.setDriveVelocity(setpoint.speedMetersPerSecond, ff);
    }
  }


  // Setpoint / state

  /**
   * Sets the desired state for this module (speed + angle), with optimization.
   *
   * @param desiredState desired SwerveModuleState
   * @param isOpenLoop   true = open-loop feedforward voltage, false = closed-loop velocity
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState optimized =
        MathSwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderDegrees()));

    // Save state for next call
    inVelocityMode = true;
    priorDriveVelocity = optimized.speedMetersPerSecond;

    setpoint = optimized;
    this.isOpenLoop = isOpenLoop;
  }

  /**
   * Sets the desired state for this module (closed-loop by default).
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, false);
  }

  /**
   * Returns the current measured state of the module (velocity + angle).
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocity(),
        Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Returns the module's current odometry position (distance + angle).
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderMeters(),
        Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Returns the current setpoint (for logging / AdvantageScope).
   */
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  // Motor mode / configuration methods

  /**
   * Sets the swerve module to coast or brake mode.
   *
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    io.setCoastMode(setCoast);
  }

  /**
   * Returns true if the drive motor is currently in coast mode.
   */
  public boolean isMotorModeCoast() {
    if (io instanceof SwerveModuleIOTalonFX talonIO) {
      return talonIO.isCoastMode();
    }
    return false;
  }

  /**
   * Sets FOC commutation on the drive motor. Takes effect on the next motor request.
   *
   * @param setFOC true = FOC mode, false = trapezoidal mode
   */
  public void setDriveMotorFOC(boolean setFOC) {
    io.setDriveFOC(setFOC);
  }

  /**
   * Re-applies full hardware configuration to this module. Used to recover from a brownout.
   * NOTE: includes blocking CAN calls.
   */
  public void configSwerveModule() {
    io.configSwerveModule();
  }

  // Direct motor output methods

  /**
   * Sets the drive motor to a direct percent output.
   *
   * @param percentOutput -1 to +1
   */
  public void setDriveMotorPercentOutput(double percentOutput) {
    setDriveMotorVoltageOutput(percentOutput * SwerveConstants.voltageCompSaturation);
  }

  /**
   * Sets the voltage output to the drive motor directly (no closed-loop).
   *
   * @param voltage nominally -12V to +12V
   */
  public void setDriveMotorVoltageOutput(double voltage) {
    io.setDriveVoltage(voltage);
    if (voltage == 0.0) {
      inVelocityMode = true;
      priorDriveVelocity = 0.0;
    } else {
      inVelocityMode = false;
    }
  }

  /**
   * Sets the turn motor to a direct percent output.
   *
   * @param percentOutput -1 to +1
   */
  public void setTurnMotorPercentOutput(double percentOutput) {
    io.setTurnPercentOutput(percentOutput);
  }

  /**
   * Sets the wheel to a specific absolute facing without optimization.
   *
   * @param angleDegrees desired wheel facing in degrees (0 = front, positive = left)
   */
  public void setWheelFacing(double angleDegrees) {
    double normalized = MathBCR.normalizeAngle(angleDegrees);
    io.setTurnPosition(Units.degreesToRadians(normalized));
  }

  /**
   * Stops both drive and turn motors.
   */
  public void stopMotors() {
    setDriveMotorVoltageOutput(0.0);  // also resets inVelocityMode + priorDriveVelocity
    setTurnMotorPercentOutput(0.0);
    inVelocityMode = true;
    priorDriveVelocity = 0.0;
  }

  // Encoder / sensor methods

  /**
   * Returns the drive encoder distance in meters (positive = forward).
   */
  public double getDriveEncoderMeters() {
    return inputs.drivePositionRad * (SwerveConstants.kWheelDiameterMeters / 2.0);
  }

  /**
   * Returns the drive encoder velocity in meters per second (positive = forward).
   */
  public double getDriveEncoderVelocity() {
    return inputs.driveVelocityRadPerSec * (SwerveConstants.kWheelDiameterMeters / 2.0);
  }

  /**
   * Returns the turn motor encoder angle in degrees (does not wrap — may exceed 360).
   */
  public double getTurningEncoderDegrees() {
    return Units.radiansToDegrees(inputs.turnPositionRad);
  }

  /**
   * Returns the turn motor encoder rotational velocity in degrees/second.
   */
  public double getTurningEncoderVelocityDPS() {
    return Units.radiansToDegrees(inputs.turnVelocityRadPerSec);
  }

  /**
   * Returns the CANcoder angle in degrees, normalized to [-180, 180).
   */
  public double getCanCoderDegrees() {
    return MathBCR.normalizeAngle(Units.radiansToDegrees(inputs.turnAbsolutePositionRad));
  }

  /**
   * Returns the CANcoder rotational velocity in degrees/second.
   */
  public double getCanCoderVelocityDPS() {
    return Units.radiansToDegrees(inputs.turnAbsoluteVelocityRadPerSec);
  }

  // Hardware information methods

  public double getDriveBusVoltage() {
    return inputs.driveSupplyVoltage;
  }

  public double getDriveOutputPercent() {
    return inputs.driveDutyCycle;
  }

  public double getDriveStatorCurrent() {
    return inputs.driveCurrentAmps;
  }

  public double getDriveTemp() {
    return inputs.driveTempCelsius;
  }

  public double getTurningOutputPercent() {
    return inputs.turnDutyCycle;
  }

  public double getTurningStatorCurrent() {
    return inputs.turnCurrentAmps;
  }

  public double getTurningTemp() {
    return inputs.turnTempCelsius;
  }

  // Elastic / logging

  /**
   * Updates relevant variables on the Elastic dashboard.
   */
  public void updateElastic() {
    SmartDashboard.putNumber(StringUtil.buildString("Swerve FXangle ", name), MathBCR.normalizeAngle(getTurningEncoderDegrees()));
    SmartDashboard.putNumber(StringUtil.buildString("Swerve CCangle ", name), getCanCoderDegrees());
    SmartDashboard.putNumber(StringUtil.buildString("Swerve FXangle dps", name), getTurningEncoderVelocityDPS());
    SmartDashboard.putNumber(StringUtil.buildString("Swerve distance", name), getDriveEncoderMeters());
    SmartDashboard.putNumber(StringUtil.buildString("Swerve drive temp ", name), getDriveTemp());
  }

  /**
   * Checks for hardware fault conditions and records sticky faults if found.
   *
   * @return true = one or more hardware faults detected
   */
  public boolean checkForFaultConditions() {
    String s = "";
    if (!inputs.driveMotorConnected) s = StringUtil.buildString(s, "-Drive");
    if (!inputs.turnMotorConnected) s = StringUtil.buildString(s, "-Turn");
    if (!inputs.CANCoderConnected) s = StringUtil.buildString(s, "-Cancoder");

    if (s.isEmpty()) return false;

    s = StringUtil.buildString("SwerveModule-", name, s);
    RobotPreferences.recordStickyFaults(s);
    DataLogUtil.writeMessageEcho(StringUtil.buildString("STICKY FAULT ", s));
    return true;
  }
}