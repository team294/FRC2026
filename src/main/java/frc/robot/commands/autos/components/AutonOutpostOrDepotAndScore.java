// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.sequences.ShooterTurretScorePrepSequence;
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
public class AutonOutpostOrDepotAndScore extends SequentialCommandGroup {
  /** Creates a new AutoOutpostOrDepotAndScore.
   * can only go to outpost from under the trench atm
   */
  public AutonOutpostOrDepotAndScore(RoutineSideType routineSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    addCommands(
      new DataLogMessage(false, "AutonOutpostOrDepotAndScore: start"),
      // If we are on the right side, we go to the outpost. Otherwise, we are on the left side and we go to the depot
      either(
        sequence(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.ShootTrenchToOutpost), driveTrain, alliance),
          new AutonScore(EndType.waitForSetPoint, intake, feeder, turret, shooter, driveTrain, field) // doesn't end
        ),
        sequence(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.ShootTrenchToDepot), driveTrain, alliance),
          new AutonScore(EndType.waitForSetPoint, intake, feeder, turret, shooter, driveTrain, field) // doesn't end

        ),
        () -> routineSide == RoutineSideType.kRightSide),
      new DataLogMessage(false, "AutonOutpostOrDepotAndScore: end")
    );
  }
}
