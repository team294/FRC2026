package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveType;
import frc.robot.utilities.AllianceSelection;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class DriveTrajectory extends SequentialCommandGroup { 
  private Pose2d initialPose; // Save initial pose for relative trajectories

  /**
   * Follows a choreo trajectory with swerve drive.
   * @param trajectoryType Specify what robot starting position to use:
   *  <p> kRelative = trajectory is in the robot coordinate system, where (0m, 0m, 0deg) is the location and facing of the robot at the start of
   *      the trajectory. +X is the direction that the robot is facing at the start of the tracjectory. Initial Pose2D in the trajectory must be (0m , 0m, 0deg).
   *  <p> kAbsolute = trajectory is in the Blue field coordinate system, where (0m, 0m) is the right corner of the Blue driver station and +X faces away from the
   *      Blue drivers. The trajectory is automatically flipped for Red if the current AllianceSelection is Red. Robot assumes its current odometry pose is
   *      correct when the trajectory starts.
   *  <p> kAbsoluteResetPose = trajectory is in the Blue field coordinate system, where (0m, 0m) is the right corner of the Blue driver station and +X faces away from the
   *      Blue drivers. The trajectory is automatically flipped for Red if the current AllianceSelection is Red. Robot odometry pose robot is reset to the 
   *      first Pose2D in the trajectory before running the trajectory.
   *  <p> kAbsoluteResetPoseTol = trajectory is in the Blue field coordinate system, where (0m, 0m) is the right corner of the Blue driver station and +X faces away from the
   *      Blue drivers. The trajectory is automatically flipped for Red if the current AllianceSelection is Red. If the current robot odometry pose is "close"
   *      (within 0.5m and 15deg) to the first Pose2D in the trajectory, then the robot current pose is assumed to be correct and is not adjusted; if it is not close, then 
   *      the odometry pose robot is reset to the first Pose2D in the trajectory before running the trajectory.
   * @param stopType Specify behavior at the end of the trajectory
   *  <p> kNoStop = Don't stop the drivetrain.  Continue the drivetrain motors in their current state (wheel orientation and velocity) when the trajectory ends.  Note that
   *      this may not exactly match the end of the planned trajectory (just because the end of the trajectory is a stop point [zero velocity], that does not mean
   *      that the robot will actually be stopped when this command ends).
   *  <p>kCoast = Stop the drive motors and set the drivetrain to Coast mode
   *  <p>kBrake = Stop the drive motors and set the drivetrain to Brake mode
   * @param trajectory swerve sample Choreo trajectory that will be run, Trajectory will be mirrored based on alliance
   * @param driveTrain DriveTrain subsystem
   * @param alliance AllianceSelection utility
   */
  public DriveTrajectory(CoordType trajectoryType, StopType stopType, Trajectory<SwerveSample> trajectory, DriveTrain driveTrain, AllianceSelection alliance) { 
    // Define the controller for robot rotation
    PIDController thetaController = new PIDController(Constants.TrajectoryConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create the serve controller command to follow the trajectory
    // Also add any initial commands before following the trajectory, depending on trajectoryType
    ChoreoFollower ChoreoFollower;
    if (trajectoryType == CoordType.kRelative) {
      // For relative trajectories, first command needs to be to save the initial robot Pose
      addCommands(new InstantCommand(() -> initialPose = driveTrain.getPose()));
    
      ChoreoFollower = new ChoreoFollower(
        stopType, 
        trajectory, 
        new PIDController(Constants.TrajectoryConstants.kPXController, 0.0, 0.0),
        new PIDController(Constants.TrajectoryConstants.kPYController, 0.0, 0.0), 
        thetaController, 
        (speeds) -> {
          Translation2d rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(initialPose.getRotation());
          driveTrain.drive(rotated.getX(), rotated.getY(), speeds.omegaRadiansPerSecond, DriveType.kFieldRelative, false);
        }, 
        // For relative trajectories, get the current pose relative to the initial robot Pose
        () -> driveTrain.getPose().relativeTo(initialPose), 
        () -> false, 
        driveTrain
      );
    } else {
      if (trajectoryType == CoordType.kAbsoluteResetPose) {
        // For AbsoluteResetPose trajectories, first command needs to be to reset the robot Pose
        addCommands(new DriveResetPose(() -> (trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get()), false, driveTrain));
      } else if (trajectoryType == CoordType.kAbsoluteResetPoseTol) {
        // For AbsoluteResetPoseTol trajectories, first command needs to be to reset the robot Pose
        addCommands(new DriveResetPose(() -> (trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get()), true, driveTrain));
      }

      ChoreoFollower = new ChoreoFollower(
        stopType,
        trajectory, 
        new PIDController(Constants.TrajectoryConstants.kPXController, 0.0, 0.0),
        new PIDController(Constants.TrajectoryConstants.kPYController, 0.0, 0.0), 
        thetaController, 
        (speeds) -> driveTrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, DriveType.kFieldRelative, false), 
        driveTrain::getPose,
        () -> alliance.getAlliance() == Alliance.Red, 
        driveTrain
      );
    }

    // Next follow the trajectory
    addCommands(ChoreoFollower);
  }
}
