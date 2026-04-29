// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberSide;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSingleStop extends Command {
  private final Climber climber;
  private final ClimberSide side;

  /**
   * Stop one climber side motor.
   *
   * @param side which climber side to stop
   * @param climber Climber subsystem
   */
  public ClimberSingleStop(ClimberSide side, Climber climber) {
    this.climber = climber;
    this.side = side;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.stopClimberMotor(side);
    DataLogUtil.writeMessage("ClimberSingleStop: Init, Side = ", side);
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

