// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCollectBalls extends SequentialCommandGroup {
  /** Collects balls from the neutral zone, either closer to the center line or along the edge closer to the alliance zone. */
  public AutonCollectBalls(BallPickupTrajType driveThroughCenter, ReturnToAllianceType returnToAllianceZone, boolean leftSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache) {
    Trajectory<SwerveSample> trajectory = driveThroughCenter == BallPickupTrajType.kDriveThroughCenter ? cache.getTrajectory(leftSide, TrajectoryName.CollectThroughCenter) : cache.getTrajectory(leftSide, TrajectoryName.CollectThroughSide);
    addCommands(
      new DataLogMessage(false, "AutonCollectBalls: start"),
      race(  
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance),
        new IntakeSequence(intake)
      ),
      new DataLogMessage(false, "AutonCollectBalls: end")
    );
  }
}
