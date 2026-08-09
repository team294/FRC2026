// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeDeployAgitateOnce extends Command {
  private final Intake intake;
  private int counter;
  
  /**
   * Agitates the intake up and down a single time, ending when done.
   * Moves the intake down if interrupted, stops rollers when done/interrupted.
   * @param intake Intake subsystem
   */
  public IntakeDeployAgitateOnce(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    intake.setIntakeRollerPercentOutput(IntakeConstants.rollerIntakePercent);
    intake.enableFastLogging(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) intake.setIntakeDeployAngle(IntakeConstants.IntakeDeployAngle.AGITATE_UPPER.value);
    else if (counter == 15) intake.setIntakeDeployAngle(IntakeConstants.IntakeDeployAngle.AGITATE_LOWER.value);

    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeDeployAngle(IntakeConstants.IntakeDeployAngle.INTAKE.value);
    intake.stopIntakeRoller();
    intake.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 15;
  }
}