// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.IntakeDeploySetAngle;
import frc.robot.commands.autos.components.AutonScore;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.Field;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPreload extends SequentialCommandGroup {
/**
 * Starts with front bumpers on the start line, with intake facing away from the driver station. 
 * If moving back, then moves backwards one meter. Shoots preload then does nothing else. 
 * @param endType end type for autonomous scoring
 * @param moveBack whether to move back or not. true = moves back one meter, false = doesn't move back
 * @param intake Intake subsystem
 * @param feeder Feeder subsystem
 * @param turret Turret subsystem
 * @param shooter Shooter subsystem
 * @param climber Climber subsystem
 * @param driveTrain DriveTrain subsystem
 * @param field Field utility
 * @param allianceSelection AllianceSelection utility
 */
  public ShootPreload(EndType endType, boolean moveBack, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, Field field, AllianceSelection allianceSelection) { 
    addCommands(
      new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 180 : 0, false, driveTrain),
      either(
        sequence(
          either(
            new DriveToPose(CoordType.kRelative, new Pose2d(1, 0, Rotation2d.kZero), driveTrain),
            none(),
            () -> moveBack
          ),
          new IntakeDeploySetAngle(IntakeDeployAngle.INTAKE, intake),
          new AutonScore(endType, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(8)
        ),
        none(),
        () -> intake.isIntakeDeployCalibrated()
      )
    );
  }
}