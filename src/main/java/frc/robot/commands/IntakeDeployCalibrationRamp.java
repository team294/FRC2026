// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

public class IntakeDeployCalibrationRamp extends Command {
  private final Intake intake;
  private double rampRate;
  private double maxPercent, currentPercent;

  /**
   * Slowly ramps voltage to the intakeDeployMotor. Used for logging data to calibrate kS and kV.
   * @param rampRate rate to increase percent output, in percent (-1.0 down to +1.0 up) per second
   * @param maxPercent stop the command when the absolute value of the percent output reaches this value
   * @param intake Intake subsystem
   */
  public IntakeDeployCalibrationRamp(double rampRate, double maxPercent, Intake intake) {
    this.intake = intake;
    this.rampRate = rampRate * 0.020; // Convert to execute() cycles = every 20msec
    this.maxPercent = maxPercent;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentPercent = 0.0;
    intake.enableFastLogging(true);
    DataLogUtil.writeMessage("IntakeDeployCalibrationRamp: Init, rampRate =", rampRate, ", maxPercent =", maxPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentPercent += rampRate;
    intake.setIntakeDeployPercentOutput(currentPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeDeploy();
    intake.enableFastLogging(false);
    DataLogUtil.writeMessage("IntakeDeployCalibrationRamp: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentPercent) >= maxPercent || 
      (currentPercent > 0 && intake.getIntakeDeployAngle() >= IntakeConstants.IntakeDeployAngle.UPPER_LIMIT.value - 10.0) ||
      (currentPercent < 0 && intake.getIntakeDeployAngle() <= IntakeConstants.IntakeDeployAngle.LOWER_LIMIT.value + 10.0);
  }
}
