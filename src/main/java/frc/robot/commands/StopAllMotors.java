// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

public class StopAllMotors extends ParallelCommandGroup {
  /** Creates a new StopAllMotors. */
  public StopAllMotors(DriveTrain driveTrain, Intake intake, Feeder feeder, Shooter shooter) {
    addCommands(
      new DriveStop(driveTrain),
      new IntakeStop(intake),
      new FeederStop(feeder),
      new ShooterStop(shooter)
    );
  }
}
