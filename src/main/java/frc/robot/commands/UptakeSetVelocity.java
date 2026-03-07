// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.FeederConstants.UptakeConstants.UptakeElasticKey.*;

public class UptakeSetVelocity extends Command {
  private final Feeder feeder;
  private double velocity = 0.0;
  private final double velocityTolerance = 50.0;
  private final boolean fromElastic;

  /**
   * Sets the target velocity for the uptakeMotor FROM ELASTIC. This command ends when the velocity is 
   * within the tolerance RPM of the target.
   * @param feeder Feeder subsystem
   */
  public UptakeSetVelocity(Feeder feeder) {
    this.feeder = feeder;
    fromElastic = true;
    addRequirements(feeder);

    if (SmartDashboard.getNumber(UPTAKE_CMD_SET_VELOCITY_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(UPTAKE_CMD_SET_VELOCITY_INPUT.value, 0.0);
    }
  }

  /**
   * Sets the target velocity for the uptakeMotor. This command ends when the velocity is 
   * within the tolerance RPM of the target.
   * @param velocity target velocity, in RPM (positive = forward, negative = reverse)
   * @param feeder Feeder subsystem
   */
  public UptakeSetVelocity(double velocity, Feeder feeder) {
    this.feeder = feeder;
    this.velocity = velocity;
    fromElastic = false;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) velocity = SmartDashboard.getNumber(UPTAKE_CMD_SET_VELOCITY_INPUT.value, 0.0);
    feeder.setUptakeVelocity(velocity);
    DataLogUtil.writeMessage("UptakeSetVelocity: Initialize, RPM =", velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DataLogUtil.writeMessage("UptakeSetVelocity: Interrupted, RPM =", feeder.getUptakeMotorVelocity(), ", Error =", feeder.getUptakeVelocityPIDError());
    } else {
      DataLogUtil.writeMessage("UptakeSetVelocity: End, RPM =", feeder.getUptakeMotorVelocity(), ", Error =", feeder.getUptakeVelocityPIDError());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(feeder.getUptakeVelocityPIDError()) < velocityTolerance;
  }
}