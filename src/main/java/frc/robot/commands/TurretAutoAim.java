// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.AllianceSelection;

public class TurretAutoAim extends Command {
  Turret turret;
  DriveTrain driveTrain;
  AllianceSelection allianceSelection;

  /**
   * Calculate target turret angle for shooting depending on robot position on field
  */
  public TurretAutoAim(Turret turret, DriveTrain driveTrain, AllianceSelection allianceSelection) {
    this.turret = turret;
    this.driveTrain = driveTrain;
    this.allianceSelection = allianceSelection;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    turret.setTurretAngle(turret.getAutoTargetAngleTurretRelative(allianceSelection.getAlliance(), driveTrain.getPose()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    turret.stopTurret();
  }
}