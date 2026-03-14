// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

public class IntakeStop extends Command {
  private final Intake intake;
  
 
  /**
   * Sets the percent output of both the Intake Deploy Motor and Intake Roller Motor to 0 and ends immediately.
   * @param intake Intake subsystem
   */
  public IntakeStop(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntakeDeploy();
    intake.stopIntakeRoller();
    DataLogUtil.writeMessage("IntakeStop: Init");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("IntakeStop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
