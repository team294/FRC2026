// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.EndType;
import frc.robot.commands.DrivePercentSpeed;
import frc.robot.commands.ShooterScorePrep;
import frc.robot.commands.sequences.FeederIntakeFeedShooterSequence.AgitateType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Field;

public class ShootOnTheMoveDebug extends SequentialCommandGroup {

  /**
   * Moves the robot 3.0meters right (RELATIVE TO ROBOT FACING) while shooting to gather data for SOTM.
   * If the turret is facing the hub, the robot will move to the left; If the turret
   * is facing the right side of the field, the robot will move toward the hub, etc.
   * @param percentSpeed percent of max speed to drive at
   * @param driveTrain DriveTrain subsystem
   * @param shooter Shooter subsystem
  */
  public ShootOnTheMoveDebug(double percentSpeed, DriveTrain driveTrain, Turret turret, Shooter shooter, Feeder feeder, Intake intake, Field field) {
    addCommands(
      new ShooterScorePrep(false, EndType.waitForSetPoint, shooter, turret, driveTrain, field),
      race(
        new DrivePercentSpeed(90.0, percentSpeed, 3.0, driveTrain),
        new FeederIntakeFeedShooterSequence(AgitateType.NONE, feeder, intake, shooter)
      )
    );
  }
}
