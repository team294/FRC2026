// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.IntakeDeploySetAngle;
import frc.robot.commands.TurretAutoAim;
import frc.robot.commands.autos.components.AutonDriveToNeutralAndCollectBalls;
import frc.robot.commands.autos.components.AutonScore;
import frc.robot.commands.sequences.AutomatedDriveAndClimbSequence;
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
public class FollowerAuto extends SequentialCommandGroup {
  /** Creates a new FollowerAuto. */
  public FollowerAuto(boolean climb, EndType shootEndType, RoutineSideType routineSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    boolean isLeftSide = routineSide == RoutineSideType.kLeftSide;
    Trajectory<SwerveSample> startTrajectory = cache.getTrajectory(isLeftSide, TrajectoryName.StartBehindTrenchSideCollectBallsFollower);
    addCommands(
      new ParallelDeadlineGroup(
        sequence(
          new DriveResetPose(startTrajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get(), isScheduled(), driveTrain),
          parallel(
            new IntakeDeploySetAngle(IntakeConstants.IntakeDeployAngle.INTAKE, intake),
            new WaitCommand(3.0)
          ),
          new AutonDriveToNeutralAndCollectBalls(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, isLeftSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(isLeftSide, TrajectoryName.CollectEndToEndFollower), driveTrain, alliance),

          new ParallelDeadlineGroup(
            either(
              new AutonScore(shootEndType, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(6.9),
              new AutonScore(shootEndType, intake, feeder, turret, shooter, climber, driveTrain, field),
              () -> climb
            ),
            either(
              new DriveToPose(CoordType.kAbsolute, () -> (new Pose2d(FieldConstants.climbPrepBlue.getX(), FieldConstants.climbPrepBlue.getY(), new Rotation2d(0))), 0.07, 1.5, driveTrain),
              new DriveToPose(CoordType.kAbsolute, () -> new Pose2d(FieldConstants.climbPrepRed.getX(), FieldConstants.climbPrepRed.getY(), Rotation2d.k180deg), 0.07, 1.5, driveTrain),
              () -> (alliance.getAlliance() == Alliance.Blue)
            )
          )
        ),
        // Turret auto aim is in parallel (deadlined) with everything above
        new TurretAutoAim(turret, driveTrain, field)
      ),

      either(
        new AutomatedDriveAndClimbSequence(alliance.getAlliance(), driveTrain, climber, shooter, turret, intake), 
        none(), 
        () -> climb
      )
    );
  }
}
