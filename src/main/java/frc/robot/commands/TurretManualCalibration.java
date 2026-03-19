// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Turret;

public class TurretManualCalibration extends InstantCommand {
  private final Turret turret;
  private final boolean manual;
  
  /**
   * Recalibrate the turret either manually (turret at absolute zero) or with the encoders (uses their measurements).
   * For the automatic, if either encoder is not connected, does not recalibrate the turret.
   * @param turret Turret subsystem
   * @param manual true = manual (assumes current physical turret rotation is at absolute zero - facing back of robot), false = calculated using encoders
   */
  public TurretManualCalibration(Turret turret, boolean manual) {
    this.turret = turret;
    this.manual = manual;
    
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (manual) {
      turret.calibrateTurret(0.0);
    } else {
      turret.calibrateTurretAutomatic();
    }
  }

  // Returns whether the command should be able to run while the robot is disabled (default = false).
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
