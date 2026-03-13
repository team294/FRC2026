// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeDeployAgitate extends Command {
  private final Intake intake;
  private int counter;
  
  /**
   * Agitates the intake up and down repeatedly, ending only if interrupted
   * and moving the intake down to intaking position when interrupted
   * @param intake Intake subsystem
   */
  public IntakeDeployAgitate(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) CommandScheduler.getInstance().schedule(new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.AGITATE_UPPER, intake));
    else if (counter == 15) CommandScheduler.getInstance().schedule(new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.AGITATE_LOWER, intake));
    else if (counter == 29) counter = -1;

    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.INTAKE, intake));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
