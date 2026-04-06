// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.LEDEventUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class IntakeRollerSetPercent extends Command {
  private final Intake intake;
  private double percent = 0.0;
  private boolean endImmediately, fromElastic;

  /**
   * Sets the percent output of the intakeDeployMotor FROM ELASTIC.
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param intake Intake subsystem
   */
  public IntakeRollerSetPercent(boolean endImmediately, Intake intake) {
    this.intake = intake;
    this.endImmediately = endImmediately;
    this.fromElastic = true;
    addRequirements(intake);

    if (SmartDashboard.getNumber(INTAKE_ROLLER_CMD_SET_PERCENT_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(INTAKE_ROLLER_CMD_SET_PERCENT_INPUT.value, 0);
    }
  }

  /**
   * Sets the percent output of the intakeDeployMotor and ends immediately.
   * @param percent output percent, -1.0 to 1.0 (positive = intake, negative = outtake)
   * @param endImmediately true = end command immediately, false = end on interrupt and stop the motor
   * @param intake Intake subsystem
   */  
  public IntakeRollerSetPercent(double percent, boolean endImmediately, Intake intake) {
    this.intake = intake;
    this.percent = percent;
    this.endImmediately = endImmediately;
    this.fromElastic = false;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromElastic) this.percent = SmartDashboard.getNumber(INTAKE_ROLLER_CMD_SET_PERCENT_INPUT.value, 0.0);
    intake.setIntakeRollerPercentOutput(percent);
    DataLogUtil.writeMessage("IntakeRollerSetPercent: Init, Percent =", percent);

    if (percent < 0) LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.REVERSE_INTAKING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (endImmediately) {
      DataLogUtil.writeMessage("IntakeRollerSetPercent: End");
    } else {
      intake.stopIntakeRoller();
      DataLogUtil.writeMessage("IntakeRollerSetPercent: End, Percent =", 0);
    }

    if (percent < 0) LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endImmediately;
  }
}
