// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutomatedDriveAndClimbSequence extends SequentialCommandGroup {

  /**
   * Automatically preps climb, drives to tower, and climbs based on current alliance.
   * Assumes current x position of robot is somewhere between tower and hub.
   * @param alliance current alliance
   * @param driveTrian DriveTrain subsystem
   * @param climber Climber subsystem
   * @param shooter Shooter subsystem
   * @param turret Turret subsystem
   * @param intake Intake subsystem
  */
  public AutomatedDriveAndClimbSequence(Alliance alliance, DriveTrain driveTrain, Climber climber, Shooter shooter, Turret turret, Intake intake) {
    addCommands(
      parallel(
        new ClimbPrepSequence(climber, shooter, turret, intake),
        either(
          new DriveToPose(CoordType.kAbsolute, new Pose2d(FieldConstants.climbPrepBlue.getX(), FieldConstants.climbPrepBlue.getY(), new Rotation2d(0)), driveTrain),
          new DriveToPose(CoordType.kAbsolute, new Pose2d(FieldConstants.climbPrepRed.getX(), FieldConstants.climbPrepRed.getY(), new Rotation2d(0)), driveTrain),
          () -> (alliance == Alliance.Blue)
        )
      ),
      either(
        new DriveToPose(CoordType.kAbsolute, new Pose2d(FieldConstants.climbExecuteBlue.getX(), FieldConstants.climbExecuteBlue.getY(), new Rotation2d(0)), driveTrain),
        new DriveToPose(CoordType.kAbsolute, new Pose2d(FieldConstants.climbExecuteRed.getX(), FieldConstants.climbExecuteRed.getY(), new Rotation2d(0)), driveTrain),
        () -> (alliance == Alliance.Blue)
      ),
      new ClimbExecuteSequence(climber)
    );
  }
}
