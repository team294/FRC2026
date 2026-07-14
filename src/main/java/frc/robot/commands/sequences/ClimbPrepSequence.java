// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.EndType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.Constants.TurretConstants.TurretAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter.Shooter;

public class ClimbPrepSequence extends ParallelCommandGroup {

  /**
   * Prep for climb sequence by raising climber, turning turret to specific angle, and bringing in intake.
   * @param climber Climber subsystem
   * @param shooter Shooter subsystem
   * @param turret Turret subsystme
   * @param intake Intake subsystem
   */
  public ClimbPrepSequence(Climber climber, Shooter shooter, Turret turret, Intake intake) {
    addCommands(
      new ClimberSetPosition(ClimberPosition.UPPER_LIMIT, climber),
      new HoodSetAngle(ShooterConstants.hoodMinimumAngle, EndType.waitForSetPoint, shooter),
      sequence(
        either(
          new TurretToggleAutoAim(turret),
          none(),
          () -> turret.getAutoAimEnabled()
        ),
        new TurretSetAngle(TurretAngle.CLIMBING_SETPOINT.value, EndType.waitForSetPoint, turret)
      ),
      new IntakeDeploySetAngle(IntakeDeployAngle.START_CONFIG, intake).raceWith(waitUntil(() -> intake.getIntakeDeployAngle() >= 0.0))
    );
  }
}
