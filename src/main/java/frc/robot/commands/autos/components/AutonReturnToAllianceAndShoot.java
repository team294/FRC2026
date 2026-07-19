// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.DriveTrajectory;
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
public class AutonReturnToAllianceAndShoot extends SequentialCommandGroup {
  /** Creates a new AutonDriveAndShoot. */
  public AutonReturnToAllianceAndShoot(ReturnToAllianceType returnToAllianceZone, AutoEndType autoEnd, boolean isSecondCycle, boolean leftSide, boolean shoot, Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain, AllianceSelection alliance, TrajectoryCache cache, Field field) {
    Trajectory<SwerveSample> trajectory = returnToAllianceZone == ReturnToAllianceType.kOverBump ? cache.getTrajectory(leftSide, TrajectoryName.CollectEndToShootBump) : cache.getTrajectory(leftSide, TrajectoryName.CollectEndToShootTrench);
    addCommands(
      parallel(
        new DataLogMessage(false, "AutonDriveAndShoot: start"),
        either(
          none(),
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance),
          () -> isSecondCycle // if it's the second cycle, we are already starting in the shoot position so no need to move
        )
      ),
      // Whether to stop and shoot or wait to shoot until at the outpost/feeder. This depends on whether we are going to the outpost and scoring from there or not
      either(
        sequence(
          // 3-28-2026:  Don changed from EndType.waitForSetPoint to EndType.runForever, which matches the teleop button
          either(
            new AutonScore(EndType.runForever, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(6.7), 
            new AutonScore(EndType.runForever, intake, feeder, turret, shooter, climber, driveTrain, field).withTimeout(10.0), 
            () -> autoEnd == AutoEndType.kSecondCycle)
           // shoot
        ),
        none(), // don't shoot
        () -> shoot),
      new DataLogMessage(false, "AutonDriveAndShoot: end")
    );
  }
}
