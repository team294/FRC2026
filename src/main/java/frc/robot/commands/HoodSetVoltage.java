// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodSetVoltage extends Command {
  private final Shooter shooter;

  private boolean fromElastic;
  private double voltage;

  /**
   * Set voltage for hood motors to move at [0.0, 4.92] with input from Elastic
   * @param shooter Shooter subsystem
   */
  public HoodSetVoltage(Shooter shooter) {
    this.shooter = shooter;
    fromElastic = true;
    addRequirements(shooter);

    if (SmartDashboard.getNumber(HOOD_CMD_SET_VOLTAGE_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(HOOD_CMD_SET_VOLTAGE_INPUT.value, 0);
    }
  }

  /**
   * Set voltage for hood motors to move at [0.0, 4.92] with input
   * @param shooter Shooter subsystem
   * @param voltage voltage to move at [0.0, 4.92]
   */
  public HoodSetVoltage(Shooter shooter, double voltage) {
    this.shooter = shooter;
    this.voltage = voltage;
    fromElastic = false;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) voltage = SmartDashboard.getNumber(HOOD_CMD_SET_VOLTAGE_INPUT.value, 0.0);
    shooter.setHoodVoltage(voltage);

    DataLogUtil.writeMessage("Hood Set Voltage: Init, Voltage =", voltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
