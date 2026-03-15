// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonOutpostOrDepotAndScore extends SequentialCommandGroup {
  /** Creates a new AutoOutpostOrDepotAndScore.
   * can only go to outpost from under the trench atm
   */
  public AutonOutpostOrDepotAndScore(RoutineSideType routineSide, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache) {
    addCommands(
      new DataLogMessage(false, "AutonOutpostOrDepotAndScore: start"),
      // If we are on the right side, we go to the outpost. Otherwise, we are on the left side and we go to the depot
      either(
        sequence(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.ShootTrenchToOutpost), driveTrain, alliance)
          // shoot ts fuel until the end of auto bc hopefully we wont run out 
        ),
        sequence(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.ShootTrenchToDepot), driveTrain, alliance)
          // shoot
        ),
        () -> routineSide == RoutineSideType.kRightSide),
      new DataLogMessage(false, "AutonOutpostOrDepotAndScore: end")
    );
  }
}
