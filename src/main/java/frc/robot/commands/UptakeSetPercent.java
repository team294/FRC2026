// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UptakeConstants.UptakeElasticKey;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.DataLogUtil;

public class UptakeSetPercent extends Command {
  private final Feeder feeder;
  private double percent = 0.0;
  private boolean endImmediately, fromElastic;

  /**
   * Sets the percent output of the uptakeMotor FROM ELASTIC.
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param feeder Feeder subsystem
   */
  public UptakeSetPercent(boolean endImmediately, Feeder feeder) {
    this.feeder = feeder;
    this.endImmediately = endImmediately;
    fromElastic = true;
    addRequirements(feeder);

    if (SmartDashboard.getNumber(UptakeElasticKey.CMD_SET_PERCENT_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(UptakeElasticKey.CMD_SET_PERCENT_INPUT.value, 0.0);
    }
  }

  /**
   * Sets the percent output of the uptakeMotor.
   * @param percent output percent, -1.0 to 1.0 (positive = forward, negative = reverse)
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param feeder Feeder subsystem
   */  
  public UptakeSetPercent(double percent, boolean endImmediately, Feeder feeder) {
    this.feeder = feeder;
    this.percent = percent;
    this.endImmediately = endImmediately;
    fromElastic = false;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = SmartDashboard.getNumber(UptakeElasticKey.CMD_SET_PERCENT_INPUT.value, 0.0);
    feeder.setUptakePercentOutput(percent);
    DataLogUtil.writeMessage("UptakeSetPercent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("UptakeSetPercent: End, Percent =", percent);
    } else {
      feeder.stopUptake();
      DataLogUtil.writeMessage("UptakeSetPercent: End, Percent =", 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}