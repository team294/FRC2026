// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.LEDEventUtil;

public class ClimberSetPercent extends Command {
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
    addRequirements(climber);

    if (SmartDashboard.getNumber(CLIMBER_CMD_SET_PERCENT_INPUT.value, -9999) == -9999) {
      SmartDashboard.putNumber(CLIMBER_CMD_SET_PERCENT_INPUT.value, 0.0);
    }

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
    DataLogUtil.writeMessage("ClimberSetPercent: Init, Percent =", percent);

    if (percent > 0) LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.CLIMBER_MOVING_UP);
    else LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.CLIMBER_MOVING_DOWN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command as soon as either side hits its corresponding limit.
    return climber.isEitherSideLimitHitForSetpoint(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If we ended due to a limit (or interruption), ensure the climber motors are stopped.
    climber.stopClimberMotors();
    DataLogUtil.writeMessage("ClimberSetPercent: End, Interupted = ", interrupted);

    LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.NEUTRAL);
  }
}
