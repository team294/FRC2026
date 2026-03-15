// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class HoodSetPercent extends Command {
  private final Shooter shooter;

  private final boolean endImmediately, fromElastic;
  private double percent;

  /**
   * Set percent for hood motors to move at [0.0, 1.0] with input from Elastic
   * @param shooter Shooter subsystem
   */
  public HoodSetPercent(boolean endImmediately, Shooter shooter) {
    this.shooter = shooter;
    this.endImmediately = endImmediately;
    fromElastic = true;

    addRequirements(shooter);

    if (SmartDashboard.getNumber(HOOD_CMD_SET_PERCENT_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(HOOD_CMD_SET_PERCENT_INPUT.value, 0);
    }
  }

  /**
   * Set percent for hood motors to move at [0.0, 1.0] with input
   * @param shooter Shooter subsystem
   * @param percent percent of max speed to move at [0.0, 1.0]
   */
  public HoodSetPercent(boolean endImmediately, double percent, Shooter shooter) {
    this.shooter = shooter;
    this.endImmediately = endImmediately;
    this.percent = percent;
    fromElastic = false;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = SmartDashboard.getNumber(HOOD_CMD_SET_PERCENT_INPUT.value, 0.0);
    shooter.setHoodVoltage(percent * ShooterConstants.hoodVoltage);

    DataLogUtil.writeMessage("Hood Set Percent: Init, Percent =", percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End");
    } else {
      shooter.setHoodVoltage(0.0);
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End, Percent =", 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}
