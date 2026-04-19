// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.subsystems.Climber;

public class ClimbExecuteSequence extends SequentialCommandGroup {

  /**
   * Climbs by moving climber down. Assumes robot is in correct position and climb has been prepped.
   * @param climber
   */
  public ClimbExecuteSequence(Climber climber) {
    addCommands(
      new ClimberSetPosition(ClimberPosition.CLIMB, climber)
    );
  }
}
