// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.FieldBoundaries;

public class DriveAlongNearestWall extends Command {
  private final DriveTrain driveTrain;
  private Pose2d currPose;
  private Rotation2d goalRotation;
  private Pose2d goalPose;
  private boolean facingLeft;
  private boolean facingBlue;

  private Command driveToPoseCommand;

  /**
   * Locates the nearest wall (either side of field, driver station wall, neutral zone, alliance zone) and drives to
   * it, rotating to prepare for fuel pickup along wall and locking joystick-controlled movement away from the wall.
   * 
   * @param driveTrain DriveTrain subsystem
  */
  public DriveAlongNearestWall(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  @Override
  public void initialize() {
    currPose = driveTrain.getPose();
    facingLeft = currPose.getRotation().getDegrees() > 0;  // BLUE alliance POV (CCW rot)
    facingBlue = Math.abs(currPose.getRotation().getDegrees()) >= 90.0;

    goalPose = currPose;

    if (currPose.getY() < FieldBoundaries.RIGHT_FIELD_WALL.yBoundaryMax && currPose.getX() > FieldBoundaries.RIGHT_FIELD_WALL.xBoundaryMin && currPose.getX() < FieldBoundaries.RIGHT_FIELD_WALL.xBoundaryMax) {
      goalRotation = facingBlue ? new Rotation2d(-1.0 * (Math.PI - FieldConstants.rotationAwayFromWall)) : new Rotation2d(-1.0 * FieldConstants.rotationAwayFromWall);
      goalPose = new Pose2d(currPose.getX(), FieldBoundaries.RIGHT_FIELD_WALL.yBoundaryMin + RobotDimensions.robotCenterToIntakeEdge, goalRotation);
      driveTrain.lockY();
      driveTrain.lockTurn();
      System.out.println("right field wall");
    }

    else if (currPose.getY() > FieldBoundaries.LEFT_FIELD_WALL.yBoundaryMin && currPose.getX() > FieldBoundaries.LEFT_FIELD_WALL.xBoundaryMin && currPose.getX() < FieldBoundaries.LEFT_FIELD_WALL.xBoundaryMax) {
      goalRotation = facingBlue ? new Rotation2d(Math.PI - FieldConstants.rotationAwayFromWall) : new Rotation2d(FieldConstants.rotationAwayFromWall);
      goalPose = new Pose2d(currPose.getX(), FieldBoundaries.LEFT_FIELD_WALL.yBoundaryMax - RobotDimensions.robotCenterToIntakeEdge, goalRotation);
      driveTrain.lockY();
      driveTrain.lockTurn();
    }

    else if (currPose.getX() < FieldBoundaries.BLUE_NEUTRAL_ZONE.xBoundaryMax && currPose.getX() > FieldBoundaries.BLUE_NEUTRAL_ZONE.xBoundaryMin) {
      goalRotation = facingLeft ? new Rotation2d(Math.PI - FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * (Math.PI - FieldConstants.rotationAwayFromWall));
      goalPose = new Pose2d(FieldBoundaries.BLUE_NEUTRAL_ZONE.xBoundaryMin + FieldConstants.hubCenterToHubEdge + RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
    }

    else if (currPose.getX() < FieldBoundaries.RED_NEUTRAL_ZONE.xBoundaryMax && currPose.getX() > FieldBoundaries.RED_NEUTRAL_ZONE.xBoundaryMin) {
      goalRotation = facingLeft ? new Rotation2d(FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * FieldConstants.rotationAwayFromWall);
      goalPose = new Pose2d(FieldBoundaries.RED_NEUTRAL_ZONE.xBoundaryMax - FieldConstants.hubCenterToHubEdge - RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
      System.out.println("red neutral zone");
    }

    else if (currPose.getX() < FieldBoundaries.BLUE_DRIVER_STATION.xBoundaryMax) {
      goalRotation = facingLeft ? new Rotation2d(Math.PI - FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * (Math.PI - FieldConstants.rotationAwayFromWall));
      goalPose = new Pose2d(FieldBoundaries.BLUE_DRIVER_STATION.xBoundaryMin + RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
    }

    else if (currPose.getX() > FieldBoundaries.RED_DRIVER_STATION.xBoundaryMin) {
      goalRotation = facingLeft ? new Rotation2d(FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * FieldConstants.rotationAwayFromWall);
      goalPose = new Pose2d(FieldBoundaries.RED_DRIVER_STATION.xBoundaryMax - RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
    }

    else if (currPose.getX() < FieldBoundaries.BLUE_ALLIANCE_ZONE.xBoundaryMax) {
      goalRotation = facingLeft ? new Rotation2d(FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * FieldConstants.rotationAwayFromWall);
      goalPose = new Pose2d(FieldBoundaries.BLUE_ALLIANCE_ZONE.xBoundaryMax - FieldConstants.hubCenterToHubEdge - RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
    }

    else if (currPose.getX() > FieldBoundaries.RED_ALLIANCE_ZONE.xBoundaryMin) {
      goalRotation = facingLeft ? new Rotation2d(Math.PI - FieldConstants.rotationAwayFromWall) : new Rotation2d(-1.0 * (Math.PI - FieldConstants.rotationAwayFromWall));
      goalPose = new Pose2d(FieldBoundaries.RED_ALLIANCE_ZONE.xBoundaryMin + RobotDimensions.robotCenterToIntakeEdge, currPose.getY(), goalRotation);
      driveTrain.lockX();
      driveTrain.lockTurn();
    }

    // actually move the robot using DriveToPose
    driveToPoseCommand = new DriveToPose(CoordType.kAbsolute, goalPose, driveTrain);
    CommandScheduler.getInstance().schedule(driveToPoseCommand);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.unlockAll();
    CommandScheduler.getInstance().cancel(driveToPoseCommand);
  }
}
