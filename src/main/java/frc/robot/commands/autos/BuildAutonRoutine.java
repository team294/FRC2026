// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.autos.components.AutonCollectBalls;
import frc.robot.commands.autos.components.AutonDriveAndShoot;
import frc.robot.commands.autos.components.AutonDriveToNeutralZone;
import frc.robot.commands.autos.components.AutonOutpostOrDepotAndScore;
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
public class BuildAutonRoutine extends SequentialCommandGroup {
  /** Creates a new BuildAutoRoutine. */
  public BuildAutonRoutine(ShootPreloadType shootPreload, BallPickupTrajType ballPickupTrajDirection, ReturnToAllianceType returnToAllianceDirection, AutoEndType autoEnd, RoutineSideType routineSide, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache) { 
    // Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber
    // intake, feeder, turret, shooter, climber, 
    boolean leftSide = routineSide == RoutineSideType.kLeftSide;
    // Initial pose: if shooting preload = inital pose of first trajectory - robot length + front bumper thickness
    //           if not shooting preload = inital pose of first trajectory
    Pose2d initialTrajectoryPose = cache.getTrajectory(leftSide, TrajectoryName.StartTrenchToCollectStart).getInitialPose(alliance.getAlliance() == Alliance.Red).get();
    Pose2d initalRobotStartPose = shootPreload == ShootPreloadType.kShootPreload ?  initialTrajectoryPose.transformBy(new Transform2d(-RobotDimensions.robotLength + RobotDimensions.frontBumperThickness, 0, Rotation2d.kZero)): initialTrajectoryPose;
    
    // If we are ending auto by going to the outpost/depot, don't shoot until at the outpost/depot (minimize time wasted)
    boolean shoot = autoEnd != AutoEndType.kOutpostOrDepot;

    addCommands(
      // First cycle
      new DriveResetPose(initalRobotStartPose, false, driveTrain),
      new AutonDriveToNeutralZone(shootPreload, leftSide, driveTrain, alliance, cache),
      new AutonCollectBalls(ballPickupTrajDirection, returnToAllianceDirection, leftSide, driveTrain, alliance, cache),
      new AutonDriveAndShoot(returnToAllianceDirection, leftSide, shoot, driveTrain, alliance, cache),
      
      // Whether we are going to do anything after the first cycle
      either(
        none(), 
        // If we are, check whether we are going to the neutral zone for a second cycle or going to the outpost/depot
        either(
          sequence( // Depending on time, might require a shorter cycle w/ shorter trajectories? 
            new AutonDriveToNeutralZone(ShootPreloadType.kDontShootPreload, leftSide, driveTrain, alliance, cache),
            // keep ball pickup trajectory same as first cycle for now
            new AutonCollectBalls(ballPickupTrajDirection, returnToAllianceDirection, leftSide, driveTrain, alliance, cache),
            new AutonDriveAndShoot(returnToAllianceDirection, leftSide, true, driveTrain, alliance, cache)
          ), 
          new AutonOutpostOrDepotAndScore(routineSide, driveTrain, alliance, cache),
          () -> autoEnd == AutoEndType.kSecondCycle 
        ), 
        () -> autoEnd == AutoEndType.kNone
      )
      // Either go to the center for a second cycle, or go to either outpost or depot

    );
  }
}
