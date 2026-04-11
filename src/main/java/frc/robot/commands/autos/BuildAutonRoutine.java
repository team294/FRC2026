// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.autos.components.AutonCollectBalls;
import frc.robot.commands.autos.components.AutonDriveToNeutralAndCollectBalls;
import frc.robot.commands.autos.components.AutonReturnToAllianceAndShoot;
import frc.robot.commands.autos.components.AutonDriveToNeutralZone;
import frc.robot.commands.autos.components.AutonOutpostOrDepotAndScore;
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
public class BuildAutonRoutine extends SequentialCommandGroup {
  /** Creates a new BuildAutoRoutine. */
  public BuildAutonRoutine(ShootPreloadType shootPreload, BallPickupTrajType ballPickupTrajDirection, ReturnToAllianceType returnToAllianceDirection, AutoEndType autoEnd, RoutineSideType routineSide, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) { 
    boolean leftSide = routineSide == RoutineSideType.kLeftSide;
    // Initial pose: if shooting preload = inital pose of first trajectory - robot length + front bumper thickness
    //           if not shooting preload = inital pose of first trajectory
    Pose2d initialTrajectoryPose = cache.getTrajectory(leftSide, TrajectoryName.StartTrenchToCollectStart).getInitialPose(alliance.getAlliance() == Alliance.Red).get();
    Pose2d initalRobotStartPose = shootPreload == ShootPreloadType.kShootPreload ?  
                                initialTrajectoryPose.transformBy(new Transform2d(-RobotDimensions.robotLength + RobotDimensions.frontBumperThickness, 0, Rotation2d.kZero)) : 
                                initialTrajectoryPose;
    
    // If we are ending auto by going to the outpost/depot, don't shoot until at the outpost/depot (minimize time wasted)
    boolean shoot = autoEnd != AutoEndType.kOutpostOrDepot;

    addCommands(
      new DataLogMessage(false, "BuildAutonRoutine: shootPreload = " + shootPreload + "ballPickupPath = " + ballPickupTrajDirection + "returnToAlliancePath = " + returnToAllianceDirection + "autoEnd = " + autoEnd + "routineSide = " + routineSide),
      // First cycle
      new DriveResetPose(initalRobotStartPose, false, driveTrain),
      new AutonDriveToNeutralAndCollectBalls(shootPreload, ballPickupTrajDirection, returnToAllianceDirection, leftSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
      // new AutonDriveToNeutralZone(returnToAllianceDirection, shootPreload, leftSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
      // new AutonCollectBalls(ballPickupTrajDirection, returnToAllianceDirection, leftSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache),
      new AutonReturnToAllianceAndShoot(returnToAllianceDirection, autoEnd, leftSide, shoot, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
      
      // Whether we are going to do anything after the first cycle
      either(
        none(), 
        // If we are, check whether we are going to the neutral zone for a second cycle or going to the outpost/depot
        either(
          sequence(
            new AutonDriveToNeutralAndCollectBalls(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kBumpSecondSwipe, returnToAllianceDirection, leftSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
            // new AutonDriveToNeutralZone(ShootPreloadType.kDontShootPreload, leftSide, intake, feeder, turret, shooter, driveTrain, alliance, cache, field),
            // // keep ball pickup trajectory same as first cycle for now
            // new AutonCollectBalls(ballPickupTrajDirection, returnToAllianceDirection, leftSide, intake, feeder, turret, shooter, driveTrain, alliance, cache),
            new AutonReturnToAllianceAndShoot(returnToAllianceDirection, autoEnd, leftSide, true, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field)
          ), 
          new AutonOutpostOrDepotAndScore(routineSide, intake, feeder, turret, shooter, climber, driveTrain, alliance, cache, field),
          () -> autoEnd == AutoEndType.kSecondCycle 
        ), 
        () -> autoEnd == AutoEndType.kNone
      )
      // Either go to the center for a second cycle, or go to either outpost or depot

    );
  }
}
