// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveStop extends Command {
  private DriveTrain drive;

  /**
   * Stops all of the drive and turn motors.
   * @param drive Drive subsystem
   */
  public DriveStop(DriveTrain drive) {
    this.drive = drive;
    
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("DriveStop: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("DriveStop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
