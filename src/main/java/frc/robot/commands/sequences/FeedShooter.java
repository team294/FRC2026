// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.commands.SpindexerSetVelocity;
import frc.robot.commands.UptakeSetPercent;
import frc.robot.commands.UptakeSetVelocity;
import frc.robot.subsystems.Feeder;


public class FeedShooter extends SequentialCommandGroup {
  /**
   * Enables uptake, then spindexer to RPMs defined in constants for shooting
   * @param feeder feeder subsystem
   */
  public FeedShooter(boolean reversed, Feeder feeder) {
    addCommands(
      either(
        sequence(
          // new UptakeSetVelocity(-1.0 * UptakeConstants.uptakeShootRPM, feeder),
          new UptakeSetPercent(-0.7, true, feeder),
          new SpindexerSetVelocity(-1.0 * SpindexerConstants.spindexerShootRPM, feeder)
        ),
        sequence(
          // new UptakeSetVelocity(UptakeConstants.uptakeShootRPM, feeder),
          new UptakeSetPercent(0.7, true, feeder),
          new SpindexerSetVelocity(SpindexerConstants.spindexerShootRPM, feeder)
        ),
        () -> reversed
      )
    );
  }
}
