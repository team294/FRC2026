// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.CLIMBER_CMD_SET_PERCENT_INPUT;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ClimberSetPercent extends InstantCommand {
  private final Climber climber;
  private final boolean fromElastic;
  private double percent;

  /**
   * Set percent of climber motors (+ is up, - is down) with input from elastic
   * @param climber Climber subsystem
   */
  public ClimberSetPercent(Climber climber) {
    this.climber = climber;
    fromElastic = true;

    if (SmartDashboard.getNumber(CLIMBER_CMD_SET_PERCENT_INPUT.value, -9999) == -9999) {
      SmartDashboard.putNumber(CLIMBER_CMD_SET_PERCENT_INPUT.value, 0.0);
    }

    addRequirements(climber);
  }

  /**
   * Set percent of climber motors (+ is up, - is down)
   * @param percent [-1.0, 1.0]
   * @param climber Climber subsystem
   */
  public ClimberSetPercent(double percent, Climber climber) {
    this.climber = climber;
    fromElastic = false;
    this.percent = percent;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = SmartDashboard.getNumber(CLIMBER_CMD_SET_PERCENT_INPUT.value, 0.0);
    climber.setClimberPercentOutput(percent);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) climber.stopClimberMotors();
  }
}
