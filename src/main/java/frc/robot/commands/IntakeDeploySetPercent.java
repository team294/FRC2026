// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeElasticKey;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

public class IntakeDeploySetPercent extends Command {
  private final Intake intake;
  private double percent = 0.0;
  private boolean endImmediately, fromElastic;

  /**
   * Sets the percent output of the intakeDeployMotor FROM ELASTIC.
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param intake Intake subsystem
   */
  public IntakeDeploySetPercent(boolean endImmediately, Intake intake) {
    this.intake = intake;
    this.endImmediately = endImmediately;
    this.fromElastic = true;
    addRequirements(intake);

    if (SmartDashboard.getNumber(IntakeElasticKey.DEPLOY_CMD_SET_PERCENT_INPUT.value, -9999.0) == -9999.0) {
      SmartDashboard.putNumber(IntakeElasticKey.DEPLOY_CMD_SET_PERCENT_INPUT.value, 0);
    }
  }

  /**
   * Sets the percent output of the intakeDeployMotor.
   * @param percent output percent, -1.0 to 1.0 (positive = deploy, negative = retract)
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param intake Intake subsystem
   */
  public IntakeDeploySetPercent(double percent, boolean endImmediately, Intake intake) {
    this.intake = intake;
    this.percent = percent;
    this.endImmediately = endImmediately;
    this.fromElastic = false;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.percent = SmartDashboard.getNumber(IntakeElasticKey.DEPLOY_CMD_SET_PERCENT_INPUT.value, 0);
    intake.setIntakeDeployPercentOutput(percent);
    DataLogUtil.writeMessage("IntakeDeploySetPercent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End");
    } else {
      intake.stopIntakeDeploy();
      DataLogUtil.writeMessage("IntakeDeploySetPercent: End, Percent =", 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}
