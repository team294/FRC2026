// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretAngle;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class Turret extends SubsystemBase {

  private final int logRotationKey;
  private final String subsystemName;

  // AdvantageKit IO layer
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Auto-aim / shoot-on-the-move state
  private boolean turretAutoAimEnabled = false;
  private boolean turretShootOnTheMove = false;

  // Current MotionMagic setpoint — tracked here for getCurrentTurretTarget()
  private double targetAngle = 0.0;
  private boolean atGoalAngle = false;

  /**
   * Constructs a Turret subsystem.
   *
   * @param io Hardware IO implementation (real or sim)
   * @param subsystemName Name (used for logging)
   */
  public Turret(TurretIO io, String subsystemName) {
    this.io = io;
    this.subsystemName = subsystemName;
    logRotationKey = DataLogUtil.allocateLogRotation();
  }

  /**
   * Gets the name of this subsystem.
   * @return subsystem name
   */
  public String getName() {
    return subsystemName;
  }


  // Turret movement methods

  /**
   * Stops the turret motor.
   */
  public void stopTurret() {
    setTurretPercentOutput(0.0);
  }

  /**
   * Sets the percent output of the turret motor.
   * Output is clamped to a reduced range when uncalibrated.
   * @param percent output percent, -1.0 to 1.0 (positive = CCW, negative = CW)
   */
  public void setTurretPercentOutput(double percent) {
    double limit = inputs.isCalibrated
        ? TurretConstants.maxPercentOutput
        : TurretConstants.maxUncalibratedPercentOutput;
    percent = MathUtil.clamp(percent, -limit, limit);
    io.setVoltage(percent * TurretConstants.compensationVoltage);
  }

  /**
   * Returns the current duty cycle of the turret motor.
   * @return duty cycle, -1.0 to 1.0
   */
  public double getTurretMotorPercentOutput() {
    return inputs.dutyCycle;
  }

  /**
   * Sets the turret to a target angle using MotionMagic closed-loop control.
   * Has no effect if the turret is not calibrated.
   * @param angle target angle, in degrees (0 = turret zero, positive = CCW, negative = CW)
   */
  public void setTurretAngle(double angle) {
    if (!inputs.isCalibrated) return;
    angle = convertToTurretRange(angle);
    io.setPosition(angle);
    targetAngle = angle;
  }

  /**
   * Normalizes an angle into the turret's valid travel range.
   * @param angle input angle, in degrees
   * @return angle wrapped into [LOWER_WRAP_POINT, UPPER_WRAP_POINT)
   */
  private double convertToTurretRange(double angle) {
    return MathUtil.inputModulus(angle, TurretAngle.LOWER_WRAP_POINT.value, TurretAngle.UPPER_WRAP_POINT.value);
  }

  /**
   * Returns the setpoint angle the turret is currently targeting.
   * Returns -999 if uncalibrated. Returns the actual turret angle if not in position control.
   * @return target angle in degrees, or -999 if uncalibrated
   */
  public double getCurrentTurretTarget() {
    if (!inputs.isCalibrated) return -999.0;
    return inputs.inPositionControl ? targetAngle : getTurretAngle();
  }

  /**
   * Returns whether the turret motor is in a position-control mode.
   * @return true = MotionMagic / PositionVoltage active, false = open-loop
   */
  public boolean isTurretMotorPositionControl() {
    return inputs.inPositionControl;
  }


  // Calibration methods

  /**
   * Calibrates the turret, declaring its current physical position to be {@code angle}.
   * Enables software limits and marks the turret as calibrated.
   * Has no effect if {@code angle} is outside the configured travel limits.
   * <p><b>NOTE:</b> includes blocking CAN calls (~50–70 ms).
   * @param angle current physical turret angle, in degrees
   */
  public void calibrateTurret(double angle) {
    io.calibrate(angle);
    // After calibration, enable auto-aim if IO reports success
    if (inputs.isCalibrated) {
      turretAutoAimEnabled = true;
      turretShootOnTheMove = true;
    }
  }

  /**
   * Marks the turret as uncalibrated, stops the motor, and disables software limits.
   * Disables auto-aim and shoot-on-the-move.
   * <p><b>NOTE:</b> includes blocking CAN calls (~50–70 ms).
   */
  public void setTurretUncalibrated() {
    io.setUncalibrated();
    turretAutoAimEnabled = false;
    turretShootOnTheMove = false;
  }

  /**
   * Attempts to re-calibrate automatically using both CANcoder readings.
   * Has no effect if either encoder is disconnected.
   * <p><b>NOTE:</b> may include blocking CAN calls (~250 ms).
   */
  public void calibrateTurretAutomatic() {
    io.recalibrateAutomatic();
    if (inputs.isCalibrated) {
      turretAutoAimEnabled = true;
      turretShootOnTheMove = true;
    }
  }

  /**
   * Returns whether the turret is calibrated.
   * @return true = calibrated, false = not calibrated
   */
  public boolean isTurretCalibrated() {
    return inputs.isCalibrated;
  }


  // CANcoder connection status

  /**
   * Returns whether the belt CANcoder is connected and reading its magnet.
   * @return true = connected and reading
   */
  public boolean isBeltEncoderConnected() {
    return inputs.beltEncoderConnected;
  }

  /**
   * Returns whether the gear CANcoder is connected and reading its magnet.
   * @return true = connected and reading
   */
  public boolean isGearEncoderConnected() {
    return inputs.gearEncoderConnected;
  }

  /**
   * Returns whether both CANcoders are connected and reading their magnets.
   * @return true = both connected
   */
  public boolean areBothCANcodersConnected() {
    return inputs.beltEncoderConnected && inputs.gearEncoderConnected;
  }


  // Encoder / position methods

  /**
   * Returns the current calibrated turret angle.
   * Returns -999 if the turret is not calibrated.
   * @return turret angle in degrees (0 = turret zero, positive = CCW, negative = CW)
   */
  public double getTurretAngle() {
    return inputs.isCalibrated ? inputs.turretPositionDeg : -999.0;
  }

  /**
   * Returns the absolute turret angle on the field as a Rotation2d.
   * @param robotRotation2d current robot heading
   * @return absolute turret heading on the field
   */
  private Rotation2d getTurretAbsoluteAngle(Rotation2d robotRotation2d) {
    double ret = robotRotation2d.getDegrees() + getTurretAngle() + 180.0;
    return Rotation2d.fromDegrees(convertToTurretRange(ret));
  }

  /**
   * Returns the absolute pose of the turret on the field.
   * @param robotPose2d current robot pose
   * @return turret Pose2d on the field
   */
  public Pose2d getTurretAbsolutePose(Pose2d robotPose2d) {
    double robotRotRadians = robotPose2d.getRotation().getRadians();
    Pose2d pose = new Pose2d(
        robotPose2d.getX() + TurretConstants.turretOffset * Math.cos(robotRotRadians),
        robotPose2d.getY() + TurretConstants.turretOffset * Math.sin(robotRotRadians),
        getTurretAbsoluteAngle(robotPose2d.getRotation()));

    Logger.recordOutput("Turret/TurretPose", pose);  // log here to avoid passing in robot pose somewhere else

    return pose;
  }

  /**
   * Computes the auto-targeting angle for a given field target (absolute frame).
   * @param robotPose2d current robot pose
   * @param targetTranslation2d field position of the aiming target
   * @return absolute field angle toward target, in degrees
   */
  private double getAutoTargetAngleAbsolute(Pose2d robotPose2d, Translation2d targetTranslation2d) {
    Pose2d turretPose = getTurretAbsolutePose(robotPose2d);
    return Math.toDegrees(Math.atan2(
        turretPose.getY() - targetTranslation2d.getY(),
        turretPose.getX() - targetTranslation2d.getX()));
  }

  /**
   * Computes the turret angle required to point at a field target, relative to turret zero.
   * @param robotPose2d current robot pose
   * @param targetTranslation2d field position of the aiming target
   * @return turret-relative target angle, in degrees
   */
  public double getAutoTargetAngleTurretRelative(Pose2d robotPose2d, Translation2d targetTranslation2d) {
    return convertToTurretRange(
        getAutoTargetAngleAbsolute(robotPose2d, targetTranslation2d)
            - robotPose2d.getRotation().getDegrees());
  }


  // Auto-aim / shoot-on-the-move

  /** Toggles whether the turret should be auto-aiming. */
  public void toggleAutoAim() {
    turretAutoAimEnabled = !turretAutoAimEnabled;
  }

  /**
   * Returns the current auto-aim state.
   * @return true = auto-aim enabled
   */
  public boolean getAutoAimEnabled() {
    return turretAutoAimEnabled;
  }

  /** Toggles shoot-on-the-move (only active when auto-aim is enabled). */
  public void toggleShootOnTheMove() {
    turretShootOnTheMove = !turretShootOnTheMove;
  }

  /**
   * Returns the current shoot-on-the-move state.
   * @return true = shoot-on-the-move enabled
   */
  public boolean getShootOnTheMoveEnabled() {
    return turretShootOnTheMove;
  }

  /**
   * Returns whether the turret is within tolerance of the most recent target angle.
   * @return true = at goal angle
   */
  public boolean isTurretAtGoalAngle() {
    return atGoalAngle;
  }


  // Periodic

  @Override
  public void periodic() {
    // Read hardware and log all inputs via AdvantageKit
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // At-goal check
    atGoalAngle = inputs.isCalibrated
        && Math.abs(targetAngle - inputs.turretPositionDeg) < TurretConstants.angleTolerance;

    // Stop motor when disabled (subsystem-level policy; IO does not know robot state)
    if (DriverStation.isDisabled()) {
      stopTurret();
    }

    // Clear auto-aim flags if calibration was lost (e.g. belt encoder dropout handled in IO)
    if (!inputs.isCalibrated) {
      turretAutoAimEnabled = false;
      turretShootOnTheMove = false;
    }

    // AdvantageKit output logging — every cycle.
    Logger.recordOutput("Turret/TurretAngle", getTurretAngle());
    Logger.recordOutput("Turret/TargetAngle", getCurrentTurretTarget());
    Logger.recordOutput("Turret/TargetSet", targetAngle);
    Logger.recordOutput("Turret/AtGoal", atGoalAngle);
    Logger.recordOutput("Turret/AutoAimEnabled", turretAutoAimEnabled);
    Logger.recordOutput("Turret/ShootOnTheMove", turretShootOnTheMove);

    // Elastic / SmartDashboard — normal rotation
    if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
      SmartDashboard.putBoolean(TURRET_IS_CANCODER_CONNECTED.value, areBothCANcodersConnected());
      SmartDashboard.putBoolean(TURRET_IS_CALIBRATED.value, inputs.isCalibrated);
      SmartDashboard.putBoolean(TURRET_IS_USING_CANCODER.value, inputs.usingBeltEncoder);
      SmartDashboard.putNumber(TURRET_CANCODER_BELT_ABS.value,  inputs.beltEncoderAbsPositionDeg);
      SmartDashboard.putNumber(TURRET_CANCODER_GEAR_ABS.value,  inputs.gearEncoderAbsPositionDeg);
      SmartDashboard.putNumber(TURRET_ANGLE.value, getTurretAngle());
      SmartDashboard.putNumber(TURRET_TARGET_ANGLE.value, getCurrentTurretTarget());
      SmartDashboard.putNumber("Turret Temp", inputs.tempCelsius);
    }

  }
}