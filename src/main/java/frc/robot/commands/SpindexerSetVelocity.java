// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants.SpindexerElasticKey;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.DataLogUtil;

public class SpindexerSetVelocity extends Command {
  private final Feeder feeder;
  private double velocity = 0.0;
  private final double velocityTolerance = 50.0;
  private boolean fromElastic;

  /**
   * Sets the target velocity for the spindexerMotor FROM ELASTIC. This command ends when the velocity is 
   * within the tolerance RPM of the target.
   * @param feeder Feeder subsystem
   */
  public SpindexerSetVelocity(Feeder feeder) {
    this.feeder = feeder;
    fromElastic = true;
    addRequirements(feeder);

    if (SmartDashboard.getNumber(SpindexerElasticKey.CMD_SET_VELOCITY_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(SpindexerElasticKey.CMD_SET_VELOCITY_INPUT.value, 0.0);
    }
  }

  /**
   * Sets the target velocity for the spindexerMotor. This command ends when the velocity is 
   * within the tolerance RPM of the target.
   * @param velocity target velocity, in RPM (positive = forward, negative = reverse)
   * @param feeder Feeder subsystem
   */
  public SpindexerSetVelocity(double velocity, Feeder feeder) {
    this.feeder = feeder;
    this.velocity = velocity;
    fromElastic = false;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) velocity = SmartDashboard.getNumber(SpindexerElasticKey.CMD_SET_VELOCITY_INPUT.value, 0.0);
    feeder.setSpindexerVelocity(velocity);
    DataLogUtil.writeMessage("SpindexerSetVelocity: Initialize, RPM =", velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DataLogUtil.writeMessage("SpindexerSetVelocity: Interrupted, RPM =", feeder.getSpindexerVelocity(), ", Error =", feeder.getSpindexerVelocityPIDError());
    } else {
      DataLogUtil.writeMessage("SpindexerSetVelocity: End, RPM =", feeder.getSpindexerVelocity(), ", Error =", feeder.getSpindexerVelocityPIDError());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(feeder.getSpindexerVelocityPIDError()) < velocityTolerance;
  }
}
