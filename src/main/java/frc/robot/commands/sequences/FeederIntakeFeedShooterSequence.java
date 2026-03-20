// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.xml.sax.helpers.ParserAdapter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.FeederFeedShooter;
import frc.robot.commands.IntakeDeployAgitate;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederIntakeFeedShooterSequence extends SequentialCommandGroup {
  /**
   * Starts feeding the shooter, and waits a bit until some balls have fed through until agitating balls with intake
   * @param feeder Feeder feeder subsystem
   * @param intake Intake intake subsystem
   */
  public FeederIntakeFeedShooterSequence(Feeder feeder, Intake intake) {
    addCommands(
      // // Ensure that all necessary subsystmes are in place
      // parallel(
      //   new WaitUntilCommand(null)
      // ),
      parallel(
      new FeederFeedShooter(feeder),
      sequence(
        waitUntil(() -> (feeder.getUptakeVelocityPIDError() < FeederConstants.uptakeVelocityToleranceToStartSpindexer))),
        new IntakeDeployAgitate(intake)
      )
    );
  }
}
