// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class ShooterSetPercent extends Command {
  private final Shooter shooter;
  
  private double percent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output of the shooter from Shuffleboard and ends immediately.
   * @param shooter Shooter subsystem
   */
  public ShooterSetPercent(Shooter shooter) {
    this.shooter = shooter;
    
    this.fromShuffleboard = true;
    addRequirements(shooter);

    if (SmartDashboard.getNumber(SHOOTER_CMD_SET_PERCENT_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(SHOOTER_CMD_SET_PERCENT_INPUT.value, 0);
    }
  }

  /**
   * Sets the percent output of the shooter and ends immediately.
   * @param percent -1.0 to 1.0 (positive = shooter, negative = reverse)
   * @param shooter Shooter subsystem
   */  
  public ShooterSetPercent(double percent, Shooter shooter) {
    this.shooter = shooter;
    
    this.percent = percent;
    this.fromShuffleboard = false;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) percent = SmartDashboard.getNumber(SHOOTER_CMD_SET_PERCENT_INPUT.value, 0.0);
    shooter.setShooterPercentOutput(percent);

    DataLogUtil.writeMessage("Shooter Set Percent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("Shooter Set Percent: End.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

