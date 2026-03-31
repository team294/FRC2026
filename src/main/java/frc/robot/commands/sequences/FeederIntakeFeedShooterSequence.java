// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.FeederFeedShooter;
import frc.robot.commands.IntakeDeployAgitate;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.DataLogUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederIntakeFeedShooterSequence extends SequentialCommandGroup {
  /**
   * Starts feeding the shooter, and waits a bit until some balls have fed through until agitating balls with intake
   * @param feeder Feeder feeder subsystem
   * @param intake Intake intake subsystem
   */
  public FeederIntakeFeedShooterSequence(Feeder feeder, Intake intake, Shooter shooter, Turret turret) {
    addCommands(
      // Ensure that all necessary subsystmes are in place
      new DataLogMessage(false, "FeederIntakeFeedShooterSequence: Init"),
      parallel(
        waitUntil(() -> shooter.isShooterAtGoalVelocity()).andThen(new DataLogMessage(false, "FeederIntakeFeedShooterSequence: ShooterAtGoal")),
        waitUntil(() -> turret.isTurretAtGoalAngle()).andThen(new DataLogMessage(false, "FeederIntakeFeedShooterSequence: TurretAtGoal")),
        waitUntil(() -> shooter.isHoodAtGoalAngle() && shooter.getHoodAngleSetpoint() != ShooterConstants.hoodMinimumAngle).andThen(new DataLogMessage(false, "FeederIntakeFeedShooterSequence: HoodAtGoal"))
      ).handleInterrupt(() -> DataLogUtil.writeMessage("FeederIntakeFeedShooterSequence: Interrupted")),
      parallel(
        new FeederFeedShooter(feeder),
        sequence(
          waitUntil(() -> (feeder.getUptakeVelocityPIDError() < FeederConstants.uptakeVelocityToleranceToStartSpindexer)),
          either(new IntakeDeployAgitate(intake), new IntakeSequence(intake), () -> intake.getAgitateDuringFeed())
        )
      ),
      new DataLogMessage(false, "FeederIntakeFeedShooterSequence: End")

    );
  }
}
