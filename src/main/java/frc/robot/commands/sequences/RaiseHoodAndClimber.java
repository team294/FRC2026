// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.EndType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.commands.HoodSetAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter.Shooter;

public class RaiseHoodAndClimber extends ParallelCommandGroup {

  /**
   * Simultaneously raise climber to extended (under tower rung, not full) config
   * and raise hood to shooting angle.
   * @param climber Climber subsystem
   * @param shooter Shooter subsystem
  */
  public RaiseHoodAndClimber(Climber climber, Shooter shooter) {
    addCommands(
      new ClimberSetPosition(ClimberPosition.UNDER_TOWER, climber),
      new HoodSetAngle(ShooterConstants.hoodShootAngle, EndType.waitForSetPoint, shooter)
    );
  }
}
