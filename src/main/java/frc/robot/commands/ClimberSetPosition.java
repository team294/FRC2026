// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElasticKeyConstants.ElasticKey;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetPosition extends Command {
  private final Climber climber;
  
  private double target;
  private double tolerance = 0.5;
  private int toleranceCount = 0;
  private boolean fromElastic;

  /**
   * Sets the target position of the climber to run a generated profile.
   * The command will end when the climber is within 0.5 inches of the target position for 5 cycles.
   * @param target target position, in inches (0 = lower limit, positive = up)
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(double target, Climber climber) {
    this.climber = climber;
    this.target = target;
    fromElastic = false;
    addRequirements(climber);
  }

  /**
   * Sets the target position of the climber to run a generated profile.
   * The command will end when the climber is within 0.5 inches of the target position for 5 cycles.
   * @param target target from ClimberPosition enum, in inches (see ClimberConstants.ClimberPosition)
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(ClimberPosition target, Climber climber) {
    this.climber = climber;
    this.target = target.value;
    fromElastic = false;
    addRequirements(climber);
  }

  /**
   * Sets the target position of the climber to run a generated profile.  Gets the target position from Suffleboard.
   * The command will end when the climber is within 0.5 inches of the target position for 5 cycles.
   * @param climber Climber subsystem
   */
  public ClimberSetPosition(Climber climber) {
    this.climber = climber;
    fromElastic = true;
    addRequirements(climber);

    if (SmartDashboard.getNumber(ElasticKey.CLIMBER_POSITION_SETPOINT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(ElasticKey.CLIMBER_POSITION_SETPOINT.value, 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) target = SmartDashboard.getNumber(ElasticKey.CLIMBER_POSITION_SETPOINT.value, 0.0);

    climber.setClimberProfileTarget(target);
    toleranceCount = 0;

    DataLogUtil.writeMessage("ClimberSetPosition: Init, Target =", target, ", Position =", climber.getClimberPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("ClimberSetPosition: End, Target =", target, ", Position =", climber.getClimberPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!climber.isClimberCalibrated() || Math.abs(climber.getClimberPosition() - target) <= tolerance) {
      toleranceCount++;
      DataLogUtil.writeMessage("ClimberSetPosition: Within Tolerance, Target =", target, ", Position =", climber.getClimberPosition(), ", Tolerance Count =", toleranceCount);
    }
    return (toleranceCount > 5);
  }
}
