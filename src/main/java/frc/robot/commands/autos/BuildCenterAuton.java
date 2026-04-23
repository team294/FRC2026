// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.IntakeDeploySetAngle;
import frc.robot.commands.TurretToggleAutoAim;
import frc.robot.commands.autos.components.AutonScore;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.commands.sequences.ShooterTurretScorePrepSequence;
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
public class BuildCenterAuton extends SequentialCommandGroup {
  /** Creates a new BuildCenterAuton. */
  public BuildCenterAuton(EndType shootEndType, boolean collectBalls, boolean climb, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, Field field, AllianceSelection alliance, TrajectoryCache cache) {
    // boolean leftSide = routineSide == RoutineSideType.kLeftSide;
    Trajectory<SwerveSample> driveToClimbTrajectory = collectBalls ? cache.getTrajectory(TrajectoryName.UnderTrenchToClimb_LabSafe) : cache.getTrajectory(TrajectoryName.CenterShootToClimb);
    Trajectory<SwerveSample> collectTrajectory = cache.getTrajectory(TrajectoryName.CenterStartCollectUnderTower_LabSafe);
    Pose2d initalPose2d = collectBalls ? collectTrajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get() : cache.getTrajectory(TrajectoryName.CenterShootToClimb).getInitialPose(alliance.getAlliance() == Alliance.Red).get().transformBy(new Transform2d(1.0, 0, Rotation2d.kZero));
    addCommands(
      new DriveResetPose(initalPose2d, true, driveTrain),
      either(
        sequence(
          // Shooting balls -- either collecting under the trench first and then shooting or just shooting preloads
          either(
            race(
              new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, collectTrajectory, driveTrain, alliance),
              sequence(
                new WaitCommand(1.0),
                new IntakeSequence(intake)
              )
            ),
            none(), //new ShootPreload(shootEndType, true, intake, feeder, turret, shooter, climber, driveTrain, field, alliance, cache), we only ever shoot from in front of tower
            () -> collectBalls
          ),
          parallel(
            new IntakeDeploySetAngle(IntakeDeployAngle.AGITATE_UPPER, intake), // can i do this
            new ShooterTurretScorePrepSequence(false, shootEndType, turret, shooter, driveTrain, field),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, driveToClimbTrajectory, driveTrain, alliance)
          ),
          new AutonScore(shootEndType, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(5.0),
          // If we climb...
          either(
            none(),// new AutomatedDriveAndClimbSequence()
            none(),
            () -> climb
          )
        ),
        none(),
        () -> intake.isIntakeDeployCalibrated()
      )
    );
  }
}
