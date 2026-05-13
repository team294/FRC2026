// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;
 
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
 
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
 
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawVelocity;
 
  public GyroIOPigeon2(int deviceId) {
    pigeon = new Pigeon2(deviceId);
 
    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
 
    // 100 Hz update rate for all signals
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, yaw, pitch, roll, yawVelocity);
    pigeon.optimizeBusUtilization();
  }
 
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity).equals(StatusCode.OK);
    inputs.yawDeg = yaw.getValueAsDouble();
    inputs.pitchDeg = pitch.getValueAsDouble();
    inputs.rollDeg = roll.getValueAsDouble();
    inputs.yawVelocityDegPerSec = yawVelocity.getValueAsDouble();
  }
 
  @Override
  public void reset() {
    pigeon.reset();
  }
}