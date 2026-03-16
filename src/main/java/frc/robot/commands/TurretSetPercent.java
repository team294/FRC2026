// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class TurretSetPercent extends Command {
  private final Turret turret;
  private double percent;
  private final boolean fromElastic;

  /**
   * Set turret percent output using a value from Elastic.
   * Command runs forever, but stops the turret if interrupted.
   * Output percent is -1.0 to 1.0 (positive = CCW, negative = CW)
   * @param turret Turret subsystem
   */
  public TurretSetPercent(Turret turret) {
    this.turret = turret;
    fromElastic = true;

    if (SmartDashboard.getNumber(TURRET_CMD_SET_PERCENT_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(TURRET_CMD_SET_PERCENT_INPUT.value, 0.0);
    }
    
    addRequirements(turret);
  }

  /**
   * Set turret percent output to specified value.
   * Command runs forever, but stops the turret if interrupted.
   * @param turret Turret subsystem
   * @param percent output percent, -1.0 to 1.0 (positive = CCW, negative = CW)
   */
  public TurretSetPercent(Turret turret, double percent) {
    this.turret = turret;
    this.percent = percent;
    fromElastic = false;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) {
      percent = SmartDashboard.getNumber(TURRET_CMD_SET_PERCENT_INPUT.value, 0.0);
    }

    turret.setTurretPercentOutput(percent);

    DataLogUtil.writeMessage("TurretSetPercent: Init, percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretPercentOutput(0.0);
    DataLogUtil.writeMessage("TurretSetPercent: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
