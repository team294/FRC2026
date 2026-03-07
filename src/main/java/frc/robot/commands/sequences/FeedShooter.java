// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.commands.SpindexerSetVelocity;
import frc.robot.commands.UptakeSetVelocity;
import frc.robot.subsystems.Feeder;


public class FeedShooter extends SequentialCommandGroup {
  /**
   * Enables uptake, then spindexer to RPMs defined in constants for shooting
   * @param feeder feeder subsystem
   */
  public FeedShooter(Feeder feeder) {
    addCommands(
      new UptakeSetVelocity(UptakeConstants.uptakeShootRPM, feeder),
      new SpindexerSetVelocity(SpindexerConstants.spindexerShootRPM, feeder)
    );
  }
}
