// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class HolonomicDriveControllerBCR {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private boolean m_firstRun = true;

  /**
   * Constructs a holonomic drive controller.
   * @param xController PID Controller to respond to error in the field-relative x direction
   * @param yController PID Controller to respond to error in the field-relative y direction
   * @param thetaController profiled PID controller to respond to error in angle
   */
  public HolonomicDriveControllerBCR(PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
  }

  /**
   * Resets the HolonomicDriveController, so that the first call to calculate()
   * resets the theta setpoint and clears the differential/integral errors.
   */
  public void reset() {
    m_firstRun = true;
  }

  /**
   * Gets whether the pose error is within tolerance of the reference.
   * @return true = pose error is within tolerance of the reference, false = not within tolerance
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   * @param tolerance pose error which is tolerable
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Gtes the next output of the holonomic drive controller in field-relative speeds.
   * @param currentPose current pose, as measured by odometry or pose estimator
   * @param trajectoryPose desired trajectory pose, as sampled for the current timestep
   *  <p><b>NOTE:</b> The rotation component of trajectoryPose is the desired velocity direction (not the desired robot facing)
   * @param desiredLinearVelocityMetersPerSecond desired linear velocity
   * @param desiredHeading desired heading (the desired robot facing)
   * @return next output of the holonomic drive controller (field-relative speeds)
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredHeading) {
    // If this is the first run, then we need to reset the theta controller to the
    // current pose's heading and reset the X and Y controllers
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_xController.reset();
      m_yController.reset();
      m_firstRun = false;
    }

    // Calculate feedforward velocities (field-relative)
    double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    double thetaFF = m_thetaController.getSetpoint().velocity;

    m_poseError = trajectoryPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    if (!m_enabled) {
      return new ChassisSpeeds(xFF, yFF, thetaFF);
    }

    // Calculate feedback velocities (based on position error)
    double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());
    double thetaFeedback = m_thetaController.calculate(currentPose.getRotation().getRadians(),
        desiredHeading.getRadians());

    // Return next output
    return new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback);
  }

  /**
   * Gest the next output of the holonomic drive controller in field-relative speeds.
   * @param currentPose current pose, as measured by odometry or pose estimator
   * @param desiredState desired trajectory pose, as sampled for the current timestep
   * <p><b>NOTE:</b> The rotation component of desiredState is the desired velocity direction (not the desired robot facing)
   * @param desiredHeading desired heading (the desired robot facing)
   * @return The next output of the holonomic drive controller (field-relative speeds)
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   * @param enabled true = controller is enabled, false = controller is disabled
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
}
