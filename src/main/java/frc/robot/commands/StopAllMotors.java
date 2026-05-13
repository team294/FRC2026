// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class StopAllMotors extends ParallelCommandGroup {
  /** Stop all motors of all subsystems
   * @param drive drive subsystem
   * @param intake intake subsystem
   * @param feeder feeder subsystem
   * @param shooter shooter subsystem
   * @param climber climber subsystem
   */
  public StopAllMotors(DriveTrain drive, Intake intake, Feeder feeder, Shooter shooter, Climber climber) {
    addCommands(
      new DriveStop(drive),
      new IntakeStop(intake),
      new FeederStop(feeder),
      new ShooterStop(shooter),
      new ClimberStop(climber)
    );
  }
}