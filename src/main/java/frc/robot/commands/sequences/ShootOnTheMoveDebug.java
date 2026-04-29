// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ShootOnTheMoveDebug extends SequentialCommandGroup {

  /**
   * Moves the robot at specified x and y velocities while shooting to gather data for SOTM.
   * Waits until velocities are hit to start shooting.
   * @param xVelocity forwards/backwards velocity, in meters/second
   * @param yVelocity left/right velocity, in meters/second
   * @param driveTrain DriveTrain subsystem
   * @param shooter Shooter subsystem
  */
  public ShootOnTheMoveDebug(double xVelocity, double yVelocity, DriveTrain driveTrain, Shooter shooter) {
    addCommands(
      // drive to pose w/ specific velocities
      // waituntil those velocities are hit
      // start shooting
      // LOG EVERYTHING!!
    );
  }
}
