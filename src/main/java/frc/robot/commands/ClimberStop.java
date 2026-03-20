// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberStop extends Command {
  private final Climber climber;

  /**
   * Stop Climber motors (it will hold its place b/c of the mechanical setup)
   * @param climber Climber subsystem
   */
  public ClimberStop(Climber climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.stopClimberMotors();
    DataLogUtil.writeMessage("ClimberStop: Init");
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
