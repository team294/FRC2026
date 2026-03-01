// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveResetDriverAngle extends InstantCommand {
  private final DriveTrain driveTrain;

  /**
   * Sets the driver's angle relative to their driver station.  This command requires that the robot is facing away from the driver when it is called.
   * This is mainly for use on partial practice fields, where the driver is sitting elsewhere.
   * @param driveTrain
   */
  public DriveResetDriverAngle(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetDriverAngle();
    DataLogUtil.writeMessage("DriveResetDriverAngle: Init, gyro =", driveTrain.getGyroRotation(), ", driver =", driveTrain.getDriverAngle());
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
