// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.*;
import frc.robot.commands.sequences.IntakeSequence;
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
public class AutonDriveToNeutralZone extends SequentialCommandGroup {
  /** Drive to the neutral zone (can only do under the trench), finishing in position to start collecting balls */
  public AutonDriveToNeutralZone(ShootPreloadType shootPreload, boolean leftSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    Trajectory<SwerveSample> trajectory = cache.getTrajectory(leftSide, TrajectoryName.StartTrenchToCollectStart);
    addCommands(
      new DataLogMessage(false, "AutonDriveToNeutralZone: start"),
        // Whether or not we shoot the preload
      either(
        sequence(
          // If shooting preload, starts with front of the robot on the start line (one robot's length back)
          new AutonScore(EndType.waitForSetPoint, intake, feeder, turret, shooter, driveTrain, field), // this currently will never end. don't shoot preload
          new DriveToPose(CoordType.kAbsolute, trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get(), driveTrain) // TODO would this make it stop? how to make it continously go into the trajectory? 
        ),
        none(),
        () -> shootPreload == ShootPreloadType.kShootPreload
      ),
      race(
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance),  // race with intake command
        sequence(
          new WaitCommand(1.0),
          new IntakeSequence(intake)
        )
      ),
      new DataLogMessage(false, "AutonDriveToNeutralZone: end")
    );
  }
}
