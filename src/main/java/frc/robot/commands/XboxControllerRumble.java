// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxControllerRumble extends InstantCommand {
  private final CommandXboxController xboxController;
  private final double rumble;

  /**
   * Set the xbox controller rumble
   * @param rumble [0.0, 1.0]
   * @param xboxController Xbox controller
   */
  public XboxControllerRumble(double rumble, CommandXboxController xboxController) {
    this.xboxController = xboxController;
    this.rumble = rumble;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xboxController.setRumble(RumbleType.kBothRumble, rumble);
  }
}
