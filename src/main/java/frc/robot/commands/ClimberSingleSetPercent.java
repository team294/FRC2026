// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;
import frc.robot.Constants.ClimberConstants.ClimberSide;

public class ClimberSingleSetPercent extends Command {
  private final Climber climber;
  private final ClimberSide side;
  private final boolean fromElastic;
  private double percent;

  /**
   * Set one climber side percent using Elastic input.  Command stops when 
   * the climber side hits the limit switch.
   *
   * @param side which climber side to control
   * @param climber Climber subsystem
   */
  public ClimberSingleSetPercent(ClimberSide side, Climber climber) {
    this.climber = climber;
    this.side = side;
    fromElastic = true;
    this.percent = 0.0;

    // Ensure Elastic input exists so SmartDashboard never returns the fallback.
    String key = side == ClimberSide.LEFT
        ? CLIMBER_CMD_SET_PERCENT_LEFT_INPUT.value
        : CLIMBER_CMD_SET_PERCENT_RIGHT_INPUT.value;

    if (SmartDashboard.getNumber(key, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(key, 0.0);
    }

    addRequirements(climber);
  }

  /**
   * Set one climber side percent directly (non-Elastic).  Command stops when 
   * the climber side hits the limit switch.
   *
   * @param percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   * @param side which climber side to control
   * @param climber Climber subsystem
   */
  public ClimberSingleSetPercent(double percent, ClimberSide side, Climber climber) {
    this.climber = climber;
    this.side = side;
    this.fromElastic = false;
    this.percent = percent;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (fromElastic) {
      String key = side == ClimberSide.LEFT
          ? CLIMBER_CMD_SET_PERCENT_LEFT_INPUT.value
          : CLIMBER_CMD_SET_PERCENT_RIGHT_INPUT.value;
      percent = SmartDashboard.getNumber(key, 0.0);
    }
    climber.setClimberPercentOutput(percent, side);
    DataLogUtil.writeMessage("ClimberSingleSetPercent: Init, Percent =", percent, ", Side = ", side);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return climber.isSideLimitHitForSetpoint(side, percent);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotor(side);
    DataLogUtil.writeMessage("ClimberSingleSetPercent: End, Side = ", side, ", Interupted = ", interrupted);
  }
}

