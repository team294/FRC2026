// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.Field;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonReturnToAllianceAndShoot extends SequentialCommandGroup {
  /** Creates a new AutonDriveAndShoot. */
  public AutonReturnToAllianceAndShoot(ReturnToAllianceType returnToAllianceZone, boolean leftSide, boolean shoot, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    addCommands(
      new DataLogMessage(false, "AutonDriveAndShoot: start"),
      // intake off
      either( // TODO replace diff trajectory with enhanced if statement?
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(leftSide, TrajectoryName.CollectEndToShootBump), driveTrain, alliance), 
        sequence(
          // make sure robot is in under-the trench-configuration. 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(leftSide, TrajectoryName.CollectEndToShootTrench), driveTrain, alliance)
        ),
        () -> returnToAllianceZone == ReturnToAllianceType.kOverBump
      ),
      // Whether to stop and shoot or wait to shoot until at the outpost/feeder. This depends on whether we are going to the outpost and scoring from there or not
      either(
        sequence(
          new AutonScore(EndType.waitForSetPoint, intake, feeder, turret, shooter, climber, driveTrain, field) // shoot
        ),
        none(), // don't shoot
        () -> shoot),
      new DataLogMessage(false, "AutonDriveAndShoot: end")
    );
  }
}
