/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.DataLogUtil;

public class DataLogMessage extends Command {
  boolean echo;
  Object[] paramArray;

  /**
   * Writes to the file log, to be used in command groups.
   * @param echo true = write to the file log and the console, false = write just to the file log
   * @param paramArray List of descriptions and values (variable number of parameters).
   * If more than one string or object is provided, then places one space between each item.
   */
  public DataLogMessage(boolean echo, Object... paramArray) {
    this.echo = echo;
    this.paramArray = paramArray;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (echo) {
      DataLogUtil.writeMessageEcho(paramArray);
    } else {
      DataLogUtil.writeMessage(paramArray);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Returns whether the command should be able to run while the robot is disabled (default = false). 
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}