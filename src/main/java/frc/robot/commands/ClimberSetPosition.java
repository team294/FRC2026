// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class ClimberSetPosition extends Command {
  private final Climber climber;
  private double targetPosition;
  private boolean fromElastic;

  /**
   * Sets the target position for the Climber and attempts to move it to that position.
   * This command ends when the position is within the tolerance of the target.
   * @param position target position, in inches
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(double position, Climber climber) {
    this.climber = climber;
    this.targetPosition = position;
    this.fromElastic = false;

    addRequirements(climber);
  }

  /**
   * Sets the target position for the Climber and attempts to move it to that position.
   * This command ends when the position is within the tolerance degrees of the target/
   * @param position target position, as a ClimberPosition enum (see ClimberConstants.ClimberPosition)
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(ClimberPosition position, Climber climber) {
    this.climber = climber;
    this.targetPosition = position.value;
    this.fromElastic = false;

    addRequirements(climber);
  }

  /**
   * Sets the target position for the Climber FROM ELASTIC and attempts to move it to that position.
   * This command ends when the position is within the tolerance degrees of the target.
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(Climber climber) {
    this.climber = climber;
    this.fromElastic = true;

    addRequirements(climber);

    if (SmartDashboard.getNumber(INTAKE_DEPLOY_CMD_SET_ANGLE_INPUT.value, -9999) == -9999) {
      SmartDashboard.putNumber(INTAKE_DEPLOY_CMD_SET_ANGLE_INPUT.value, ClimberPosition.TRENCH_SAFE.value);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.targetPosition = SmartDashboard.getNumber(INTAKE_DEPLOY_CMD_SET_ANGLE_INPUT.value, ClimberPosition.TRENCH_SAFE.value);
    climber.setClimberPosition(targetPosition);

    climber.enableFastLogging(true);
    DataLogUtil.writeMessage("ClimberSetPosition: Init, Target =", targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.enableFastLogging(false);
    if (interrupted) DataLogUtil.writeMessage("ClimberSetPosition: Interrupted, Target =", targetPosition, ", Current =", climber.getClimberPosition());
    else DataLogUtil.writeMessage("ClimberSetPosition: End, Target =", targetPosition, ", Current =", climber.getClimberPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.getClimberPosition() - climber.getClimberTargetPosition()) < ClimberConstants.setPositionTolerance;
  }
}
