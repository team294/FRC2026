// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveToggleCoastMode extends Command {
  private final DriveTrain driveTrain;
  

  /**
   * Toggle between coast mode and brake mode on the driveTrain.
   * @param driveTrain DriveTrain subsystem
   */
  public DriveToggleCoastMode(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driveTrain.isDriveModeCoast()) {
      driveTrain.setDriveModeCoast(false);
    } else {
      driveTrain.setDriveModeCoast(true);
    }

    DataLogUtil.writeMessage("DriveToggleCoastMode: Init, Coast Mode =", driveTrain.isDriveModeCoast());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("DriveToggleCoastMode: End, Coast Mode =", driveTrain.isDriveModeCoast());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
