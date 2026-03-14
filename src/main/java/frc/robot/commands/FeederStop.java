// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.DataLogUtil;

public class FeederStop extends Command {
  private final Feeder feeder;
  
 
  /**
   * Sets the percent output of both the uptakeMotor and spindexerMotor to 0 and ends immediately.
   * @param feeder Feeder subsystem
   */
  public FeederStop(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.stopSpindexer();
    feeder.stopUptake();
    DataLogUtil.writeMessage("FeederStop: Init");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("FeederStop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
