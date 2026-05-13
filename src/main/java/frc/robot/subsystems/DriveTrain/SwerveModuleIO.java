// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveMotorConnected = false;
    public boolean turnMotorConnected = false;
    public boolean CANCoderConnected = false;

    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveSupplyVoltage = 0.0;
    public double driveDutyCycle = 0.0;
    public double driveTempCelsius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAbsoluteVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnDutyCycle = 0.0;
    public double turnTempCelsius = 0.0;
  }

  /** Updates the logged inputs. Must be called periodically. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run drive motor at specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run turn motor at specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable drive and turn motor brake mode. */
  public default void setCoastMode(boolean coast) {}

  /** Get whether drive and turn motors are in coast mode. (false = brake mode) */
  public default boolean isCoastMode() {return false;}

  /** Run drive motor at specified percent. */
  public default void setDrivePercentOutput(double percent) {}

  /** Run turn motor at specified percent. */
  public default void setTurnPercentOutput(double percent) {}

  /** Set turn motor to specified rotation. */
  public default void setTurnPosition(double position) {}

  /** Set drive motor to specified velocity. */
  public default void setDriveVelocity(double velocity, double ff) {}

  /** Set enabled/disabled state of FOC on drive motor. */
  public default void setDriveFOC(boolean enabled) {}

  /** Set enabled/disabled state of FOC on turn motor. */
  public default void setTurnFOC(boolean enabled) {}

  /**
   * Configures the motors and encoders, uses default values from the constructor.
   * In general, this should only be called from the constructor when the robot code is starting.
   * However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   * <p><b>NOTE:</b> this procedure includes multiple blocking calls and will delay robot code.
   */
  public default void configSwerveModule() {}
}