// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElasticKeyConstants.ElasticKey;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetPercent extends Command {
  private final Climber climber;
  
  private double percent = 0.0;
  private boolean fromElastic;
  private boolean endImmediately;

  /**
   * Sets the percent output of the climber and ends immediately.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param climber Climber subsystem
   */
  public ClimberSetPercent(double percent, boolean endImmediately, Climber climber) {
    this.percent = percent;
    this.endImmediately = endImmediately;
    this.climber = climber;
    
    this.fromElastic = false;
    addRequirements(climber);
  }

  /**
   * Sets the percent output of the climber from Elastic and ends immediately.
   * @param climber Climber subsystem
   */
  public ClimberSetPercent(boolean endImmediately, Climber climber) {
    this.endImmediately = endImmediately;
    this.climber = climber;
    
    this.fromElastic = true;
    addRequirements(climber);

    if (SmartDashboard.getNumber(ElasticKey.CLIMBER_PERCENT_SETPOINT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(ElasticKey.CLIMBER_PERCENT_SETPOINT.value, 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) percent = SmartDashboard.getNumber(ElasticKey.CLIMBER_PERCENT_SETPOINT.value, 0.0);
    climber.setClimberPercentOutput(percent);

    DataLogUtil.writeMessage("ClimberSetPercent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!endImmediately || interrupted) climber.stopClimberMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endImmediately) return true;
    else return false;
  }
}
