// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class VisionOdometryStateSet extends InstantCommand {
  private final boolean enabled;
  private final DriveTrain driveTrain;
  

  /**
   * Turns on or off vision updates for odometry.
   * @param enabled true = uses vision for odometry, false = does not use vision for odometry   
   * @param driveTrain DriveTrain subsystem
   */
  public VisionOdometryStateSet(boolean enabled, DriveTrain driveTrain) {
    this.enabled = enabled;
    this.driveTrain = driveTrain;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setVisionForOdometryState(enabled);
    DataLogUtil.writeMessage("VisionOdometryStateSet: Init, Enabled =", enabled);
  }
  
  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
