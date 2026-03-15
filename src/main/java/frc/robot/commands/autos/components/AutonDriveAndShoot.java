// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonDriveAndShoot extends SequentialCommandGroup {
  /** Creates a new AutonDriveAndShoot. */
  public AutonDriveAndShoot(ReturnToAllianceType returnToAllianceZone, boolean leftSide, boolean shoot, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache) {
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
      // Whether to stop and shoot or wait to shoot until at the outpost/feeder
      either(
        none(), // shoot
        none(), // don't shoot
        () -> shoot),
      new DataLogMessage(false, "AutonDriveAndShoot: start")
    );
  }
}
