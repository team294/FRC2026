// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class IntakeDeployCalibrateManual extends InstantCommand {
  private final Intake intake;
  private double angle;
  private boolean fromElastic;

  /**
   * Manually calibrate the intakeDeployMotor with a known current angle.
   * @param angle the known current angle, in degrees
   * @param intake Intake subsystem
   */
  public IntakeDeployCalibrateManual(double angle, Intake intake) {
    this.intake = intake;
    this.angle = angle;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Manually calibrate the intakeDeployMotor with a known current angle FROM ELASTIC.
   * @param intake Intake subsystem
   */
  public IntakeDeployCalibrateManual(Intake intake) {
    this.intake = intake;
    this.fromElastic = true;
    addRequirements(intake);

    if (SmartDashboard.getNumber(INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL_INPUT.value, -9999) == -9999) {
      SmartDashboard.putNumber(INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL_INPUT.value, IntakeDeployAngle.INTAKE.value);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.angle = SmartDashboard.getNumber(INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL_INPUT.value, IntakeDeployAngle.UPPER_LIMIT.value);
    intake.calibrateIntakeDeployEncoder(angle);
    DataLogUtil.writeMessage("IntakeDeployCalibrateManual: Init, Angle =", angle);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
