// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants.IntakeDeployAngle;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.FeederStop;
import frc.robot.commands.IntakeDeploySetAngle;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.TurretAutoAim;
import frc.robot.commands.autos.components.AutonScore;
import frc.robot.commands.sequences.AutomatedDriveAndClimbSequence;
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
    Trajectory<SwerveSample> driveToClimbTrajectory = collectBalls ? cache.getTrajectory(TrajectoryName.CollectDepotToClimb_LabSafe) : cache.getTrajectory(TrajectoryName.CenterShootToClimb);
    Trajectory<SwerveSample> collectTrajectory = cache.getTrajectory(TrajectoryName.CenterStartCollectUnderTower_LabSafe);
    Pose2d initalPose2d = collectBalls ? collectTrajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get() : cache.getTrajectory(TrajectoryName.CenterShootToClimb).getInitialPose(alliance.getAlliance() == Alliance.Red).get().transformBy(new Transform2d(1.0, 0, Rotation2d.kZero));
    addCommands(
      new DriveResetPose(initalPose2d, true, driveTrain),
      either(
        sequence(
          // Shooting balls -- either collecting under the trench first and then shooting or just shooting preloads
          new ParallelDeadlineGroup(
            sequence(
              // Drive to collect balls (if collectBalls = true)
              either(
                parallel(
                  new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, collectTrajectory, driveTrain, alliance),
                  sequence(
                    new WaitCommand(1.0),
                    new IntakeSequence(intake)
                  )
                ),
                none(), 
                () -> collectBalls
              ),

              // Drive to shooting position.  Prep in parallel (deadline - continue to next section when driving is complete)
              new ParallelDeadlineGroup(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, driveToClimbTrajectory, driveTrain, alliance),
                new IntakeDeploySetAngle(IntakeDeployAngle.AGITATE_UPPER, intake),
                new ShooterTurretScorePrepSequence(false, shootEndType, turret, shooter, driveTrain, field)
              ),

              // Score while adjusting robot location to prep-to-climb position
              new ParallelDeadlineGroup(
                either(
                  new AutonScore(shootEndType, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(4.5),
                  new AutonScore(shootEndType, intake, feeder, turret, shooter, climber, driveTrain, field),
                  () -> climb
                ),
                either(
                  new DriveToPose(CoordType.kAbsolute, () -> (new Pose2d(FieldConstants.climbPrepBlue.getX(), FieldConstants.climbPrepBlue.getY(), new Rotation2d(0))), 0.07, 1.5, driveTrain),
                  new DriveToPose(CoordType.kAbsolute, () -> new Pose2d(FieldConstants.climbPrepRed.getX(), FieldConstants.climbPrepRed.getY(), Rotation2d.k180deg), 0.07, 1.5, driveTrain),
                  () -> (alliance.getAlliance() == Alliance.Blue)
                )
              ),

              // Stop shooting
              parallel(
                new ShooterStop(shooter),
                new FeederStop(feeder)
              )
            ),

            // Turret auto aim is in parallel (deadlined) with everything above
            new TurretAutoAim(turret, driveTrain, field)
          ),

          // If we climb...
          either(
            new AutomatedDriveAndClimbSequence(alliance.getAlliance(), driveTrain, climber, shooter, turret, intake),
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
