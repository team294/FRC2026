// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

  private double yawDeg = 0.0;
  private double yawVelocityDegPerSec = 0.0;

  public GyroIOSim() {}

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawDeg = yawDeg;
    inputs.pitchDeg = 0.0;
    inputs.rollDeg = 0.0;
    inputs.yawVelocityDegPerSec = yawVelocityDegPerSec;
  }

  @Override
  public void reset() {
    yawDeg = 0.0;
    yawVelocityDegPerSec = 0.0;
  }

  /**
   * Updates the simulated gyro state from the drivetrain's simulated rotation rate.
   * Call this every periodic loop from Drive.simulationPeriodic(), passing in the
   * chassis angular velocity derived from the swerve drive simulation.
   *
   * @param rotationRateDegPerSec Chassis rotation rate in degrees/second (positive = counterclockwise)
   * @param dtSeconds             Loop period in seconds (typically 0.02)
   */
  public void updateSimState(double rotationRateDegPerSec, double dtSeconds) {
    yawVelocityDegPerSec = rotationRateDegPerSec;
    yawDeg += rotationRateDegPerSec * dtSeconds;
  }

  /**
   * Sets the simulated yaw directly. Useful for teleporting the simulated robot
   * to a known heading (e.g. at the start of a test or auto routine).
   *
   * @param yawDegrees Desired yaw in degrees
   */
  public void setYaw(double yawDegrees) {
    this.yawDeg = yawDegrees;
  }

  /**
   * Returns the current simulated yaw as a Rotation2d.
   */
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(yawDeg);
  }
}