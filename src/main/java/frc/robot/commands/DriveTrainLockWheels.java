// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainLockWheels extends Command {
  private final DriveTrain driveTrain;

  /**
   * Angles DriveTrain wheels to resist pushing as much as possible.
  */
  public DriveTrainLockWheels(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setWheelFacing(0, 45);
    driveTrain.setWheelFacing(1, -45);
    driveTrain.setWheelFacing(2, -45);
    driveTrain.setWheelFacing(3, 45);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
