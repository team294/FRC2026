// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberSingleStop extends Command {
  private final Climber climber;
  // Side convention: 1 = left, 2 = right
  private final int side;

  /**
   * Stop one climber side motor.
   *
   * @param side which climber side to stop
   * @param climber Climber subsystem
   */
  public ClimberSingleStop(int side, Climber climber) {
    this.climber = climber;
    this.side = side;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.stopClimberMotor(side);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}

