// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainSetFineControl extends InstantCommand {
  private final DriveTrain driveTrain;
  private final boolean enabled;

  /**
   * Set fine control on drivetrain.
   * @param enabled true = fine control (slow movement), false = normal control
   * @param driveTrain DriveTrain subsystem
   */
  public DriveTrainSetFineControl(boolean enabled, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    this.enabled = enabled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setFineControl(enabled);
  }
}
