// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.EndType;
import frc.robot.commands.ShooterScorePrep;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Field;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterTurretScorePrepSequence extends SequentialCommandGroup {
  /**
   * Preps the shooter + hood for scoring based on current position, and ensures that turret is at correct angle 
   * @param goalPoseSupplier either Field.getStaticBallTarget() or Field.getGhostBallTarget() based on wanted target
   * @param turret Turret turret subsystem
   * @param shooter Shooter shooter subsystem 
   * @param driveTrain DriveTrain driveTrain -- only for current pose information
   */
  public ShooterTurretScorePrepSequence(boolean shootOnTheMove, EndType endType, Turret turret, Shooter shooter, DriveTrain driveTrain, Field field) {
    addCommands(
      parallel(
        new ShooterScorePrep(shootOnTheMove, endType, shooter, turret, driveTrain, field), 
        new WaitUntilCommand(() -> turret.isTurretAtGoalAngle())
        )
    );
  }
}
