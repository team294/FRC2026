// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.LEDEventUtil;

public class IntakeSequence extends SequentialCommandGroup {
  /**
   * Deploys intake and spins rollers to start intaking pieces
   * @param intake intake subsystem
   */
  public IntakeSequence(Intake intake) {
    addCommands(
      new DataLogMessage(false, "IntakeSequence: Start"),
      new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.INTAKE, intake).withTimeout(0.6),
      new IntakeRollerSetPercent(IntakeConstants.rollerIntakePercent, true, intake),
      // new InstantCommand(() -> LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.INTAKING)),
      new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.INTAKE, intake),
      new DataLogMessage(false, "IntakeSequence: End")
    );
  }
}
