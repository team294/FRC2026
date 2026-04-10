// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.commands.ClimberSetPercent;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.HoodSetAngle;
import frc.robot.commands.IntakeRollerSetPercent;
import frc.robot.commands.sequences.IntakeSequence;
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
public class AutonDriveToNeutralAndCollectBalls extends SequentialCommandGroup {
  /** Creates a new AutonDriveToNeutralAndCollectBalls. */
  public AutonDriveToNeutralAndCollectBalls(ShootPreloadType shootPreload, BallPickupTrajType driveThroughCenter, ReturnToAllianceType returnToAllianceZone, boolean leftSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    Trajectory<SwerveSample> trajectory = driveThroughCenter == BallPickupTrajType.kDriveThroughCenter ? cache.getTrajectory(leftSide, TrajectoryName.StartTrenchCenterCollectBalls) : cache.getTrajectory(leftSide, TrajectoryName.StartTrenchSideCollectBalls);

    addCommands(
      new DataLogMessage(false, "AutonDriveToNeutralAndCollectBalls: start"),

      // Whether or not we shoot the preload
      either(
        sequence(
          // If shooting preload, starts with front of the robot on the start line (one robot's length back)
          new AutonScore(EndType.waitForSetPoint, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(1.0),
          new DriveToPose(CoordType.kAbsolute, trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get(), driveTrain) // TODO would this make it stop? how to make it continously go into the trajectory? 
        ),
        none(),
        () -> shootPreload == ShootPreloadType.kShootPreload
      ),
      deadline(
        new DriveTrajectory(CoordType.kAbsolute, StopType.kNoStop, trajectory, driveTrain, alliance),  // race with intake command
        sequence(
          new WaitCommand(1.0),
          either(
            parallel(
              new IntakeSequence(intake),
              new ClimberSetPercent(ClimberConstants.maxManualPercentOutput, climber), 
              new HoodSetAngle(ShooterConstants.hoodShootAngle, EndType.waitForSetPoint, shooter)
            ),
            new IntakeSequence(intake),
            () -> returnToAllianceZone == ReturnToAllianceType.kOverBump
          )
        )
      ),
      new DataLogMessage(false, "AutonDriveToNeutralAndCollectBalls: end")
    );
  }
}
