// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;
 
import org.littletonrobotics.junction.AutoLog;
 
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawDeg = 0.0;
    public double pitchDeg = 0.0;
    public double rollDeg = 0.0;
    public double yawVelocityDegPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}
}
