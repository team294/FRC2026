// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.Constants.IntakeConstants.IntakeElasticKey;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;

public class IntakeDeploySetAngle extends Command {
  private final Intake intake;
  private double targetAngle;
  private final double angleTolerance = 3.0;
  private boolean fromElastic;

  /**
   * Sets the target angle for the intakeDeployMotor and attempts to move it to that angle. It is constrained by
   * calibration and interlocks. This command ends when the position is within the tolerance degrees of the target,
   * or if uncalibrated, immediately.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(double angle, Intake intake) {
    this.intake = intake;
    this.targetAngle = angle;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Sets the target angle for the intakeDeployMotor and attempts to move it to that angle. It is constrained by
   * calibration and interlocks. This command ends when the position is within the tolerance degrees of the target,
   * or if uncalibrated, immediately.
   * @param position target angle, in degrees (see IntakeConstants.IntakeDeployAngle)
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(IntakeDeployAngle angle, Intake intake) {
    this.intake = intake;
    this.targetAngle = angle.value;
    this.fromElastic = false;
    addRequirements(intake);
  }

  /**
   * Sets the target angle for the intakeDeployMotor FROM ELASTIC and attempts to move it to that angle. It is
   * constrained by calibration and interlocks. This command ends when the position is within the tolerance degrees
   * of the target, or if uncalibrated, immediately.
   * @param intake Intake subsystem
   */
  public IntakeDeploySetAngle(Intake intake) {
    this.intake = intake;
    this.fromElastic = true;
    addRequirements(intake);

    if (SmartDashboard.getNumber(IntakeElasticKey.DEPLOY_CMD_SET_ANGLE_INPUT.value, -9999) == -9999) {
      SmartDashboard.putNumber(IntakeElasticKey.DEPLOY_CMD_SET_ANGLE_INPUT.value, IntakeDeployAngle.START_CONFIG.value);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.targetAngle = SmartDashboard.getNumber(IntakeElasticKey.DEPLOY_CMD_SET_ANGLE_INPUT.value, IntakeDeployAngle.START_CONFIG.value);
    intake.setIntakeDeployAngle(targetAngle);
    DataLogUtil.writeMessage("IntakeDeploySetAngle: Init, Target =", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) DataLogUtil.writeMessage("IntakeDeploySetAngle: Interrupted, Target =", targetAngle, ", Current =", intake.getIntakeDeployAngle());
    else DataLogUtil.writeMessage("IntakeDeploySetAngle: End, Target =", targetAngle, ", Current =", intake.getIntakeDeployAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.isIntakeDeployCalibrated() || Math.abs(intake.getIntakeDeployAngle() - intake.getIntakeDeployTargetAngle()) < angleTolerance;
  }
}
