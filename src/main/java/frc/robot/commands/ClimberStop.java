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
   * Sets the percent output of the climberMotors to 0 and ends immediately.
   * @param climber Climber subsystem
   */
  public ClimberStop(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setClimberPercentOutput(0);
    DataLogUtil.writeMessage("ClimberStop: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("ClimberStop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
