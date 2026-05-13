// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrain.DriveType;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.HolonomicDriveControllerBCR;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.Translation2dBCR;
import frc.robot.utilities.TrapezoidProfileBCR;

public class DriveToPose extends Command {
  private final DriveTrain driveTrain;
  
  
  private final Timer timer = new Timer();
  private HolonomicDriveControllerBCR controller;
  private boolean openLoopSwerve = false; // true = turn off feedback on swerve modules

  private double maxThetaErrorDegrees = TrajectoryConstants.maxThetaErrorDegrees;      
  private double maxPositionErrorMeters = TrajectoryConstants.maxPositionErrorMeters;   

  // Options to control how the goal is specified
  private enum GoalMode {
    pose, poseSupplier, angleRelative, angleAbsolute, shuffleboard
  }

  private final GoalMode goalMode;
  private final CoordType poseType;
  private TrapezoidProfileBCR.Constraints trapProfileConstraints;
  private Pose2d inputPose;                 // Pose input directly to command
  private Supplier<Pose2d> goalSupplier;    // Supplier for goalPose
  private Rotation2d rotation;              // Rotation for goalPose
  private Pose2d initialPose, goalPose;     // Starting and destination robot pose (location and rotation) on the field
  private Translation2d initialTranslation; // Starting robot translation on the field
  private Translation2d goalDirection;      // Unit vector pointing from initial pose to goal pose = direction of travel
  private TrapezoidProfileBCR profile;      // Relative linear distance/speeds from initial pose to goal pose 

  private Translation2d curRobotTranslation;  // Current robot translation relative to initialTranslation

  private SendableChooser<Integer> feedbackChooser = new SendableChooser<>(); // Shuffleboard chooser for turning parameters on/off for tuning the drive base

	private static final int FEEDBACK_NORMAL = 0;         // Use position and velocity feedback
	private static final int FEEDBACK_VELOCITY_ONLY = 1;  // Use velocity feedback only, do not use position feedback
	private static final int FEEDBACK_NONE = 2;           // Turn off velocity and position feedback

  private final DataLog log = DataLogManager.getLog();
  private final StringLogEntry dLogTrajType = new StringLogEntry(log, "/DrivePathFollower/trajType");
  private final StructLogEntry<Pose2d> dLogCurPose2D = StructLogEntry.create(log, "/DrivePathFollower/curPose2d", Pose2d.struct);
  private final StructLogEntry<Pose2d> dLogTrajPose2D = StructLogEntry.create(log, "/DrivePathFollower/trajPose2d", Pose2d.struct);
  private final DoubleLogEntry dLogTime = new DoubleLogEntry(log, "/DrivePathFollower/time");
  private final DoubleLogEntry dLogTrajX = new DoubleLogEntry(log, "/DrivePathFollower/trajX");
  private final DoubleLogEntry dLogTrajY = new DoubleLogEntry(log, "/DrivePathFollower/trajY");
  private final DoubleLogEntry dLogTrajAccel = new DoubleLogEntry(log, "/DrivePathFollower/trajAccel");
  private final DoubleLogEntry dLogTrajVel = new DoubleLogEntry(log, "/DrivePathFollower/trajVel");
  private final DoubleLogEntry dLogTrajVelkA = new DoubleLogEntry(log, "/DrivePathFollower/trajVelkA");
  private final DoubleLogEntry dLogTrajVelAng = new DoubleLogEntry(log, "/DrivePathFollower/trajVelAng");
  private final DoubleLogEntry dLogTrajRot = new DoubleLogEntry(log, "/DrivePathFollower/trajRot");
  private final DoubleLogEntry dLogRobotPosErr = new DoubleLogEntry(log, "/DrivePathFollower/robotPosErr");
  private final DoubleLogEntry dLogRobotThErr = new DoubleLogEntry(log, "/DrivePathFollower/robotThErr");
  private final DoubleLogEntry dLogRobotX = new DoubleLogEntry(log, "/DrivePathFollower/robotX");
  private final DoubleLogEntry dLogRobotY = new DoubleLogEntry(log, "/DrivePathFollower/robotY");
  private final DoubleLogEntry dLogRobotVel = new DoubleLogEntry(log, "/DrivePathFollower/robotVel");
  private final DoubleLogEntry dLogRobotXVel = new DoubleLogEntry(log, "/DrivePathFollower/robotXVel");
  private final DoubleLogEntry dLogRobotYVel = new DoubleLogEntry(log, "/DrivePathFollower/robotYVel");
  private final DoubleLogEntry dLogRobotVelAng = new DoubleLogEntry(log, "/DrivePathFollower/robotVelAng");
  private final DoubleLogEntry dLogRobotRot = new DoubleLogEntry(log, "/DrivePathFollower/robotRot");
  private final DoubleLogEntry dLogRobotPitch = new DoubleLogEntry(log, "/DrivePathFollower/robotPitch");

  


  /**
   * Drives the robot to the desired pose in field-relative or robot-relative coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param poseType Determines how goalPose is interpreted:
   *  <p> kRelative = goalPose is in the robot coordinate system
   *  <p> kAbsolute = goalPose is in the Blue field coordinate system, based on the Blue alliance. The goal pose is unaffected by the currently selected alliance (red or blue).
   *  <p> kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use. Behaves same as kAbsolute.
   * @param goalPose target pose. Pose components include
   *  <p> For poseType = kRelative:
   *  <p> Robot forward movement, in meters (0 = current robot location, + = robot forward, - = robot reverse)
   *  <p> Robot left movement, in meters (0 = current robot location, + = robot left, - = robot right)
   *  <p> Robot angle, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   *  <p> For poseType = kAbsolute:
   *  <p> Desired X location in the field, in meters (0 = field edge in front of the Blue driver station, + = away from the Blue drivestation)
   *  <p> Desired Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, + = left when looking from the Blue drivestation)
   *  <p> Desired angle on the field, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   */
   public DriveToPose(CoordType poseType, Pose2d goalPose, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.poseType = poseType;
    inputPose = goalPose;
    goalMode = GoalMode.pose;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose in field-relative or robot-relative coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param poseType Determines how goalPose is interpreted:
   *  <p> kRelative = goalPose is in the robot coordinate system
   *  <p> kAbsolute = goalPose is in the Blue field coordinate system, based on the Blue alliance. The goal pose is unaffected by the currently selected alliance (red or blue).
   *  <p> kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use. Behaves same as kAbsolute.
   * @param goalPoseSupplier A function that supplies the target pose. Pose components include
   *  <p> For poseType = kRelative:
   *  <p> Robot forward movement, in meters (0 = current robot location, + = robot forward, - = robot reverse)
   *  <p> Robot left movement, in meters (0 = current robot location, + = robot left, - = robot right)
   *  <p> Robot angle, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   *  <p> For poseType = kAbsolute:
   *  <p> Desired X location in the field, in meters (0 = field edge in front of the Blue driver station, + = away from the Blue drivestation)
   *  <p> Desired Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, + = left when looking from the Blue drivestation)
   *  <p> Desired angle on the field, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   * @param maxVelMetersPerSecond max velocity to drive, in meters per second
   * @param maxAccelMetersPerSecondSquare max acceleration/deceleration, in meters per second squared
   * @param maxPositionErrorMeters tolerance for end position in meters
   * @param maxThetaErrorDegrees tolerance for end theta in degrees
   * @param closedLoopSwerve true = use velocity control feedback+feedforward on swerve motors (normal), false = use feed-forward only on swerve motors (test mode)
   * @param usePositionFeedback true = use position feedback to keep robot movement accuracy (normal), false = turn off postion feedback (test mode)
   * @param driveTrain DriveTrain subsystem
   */
  public DriveToPose(CoordType poseType, Supplier<Pose2d> goalPoseSupplier, double maxVelMetersPerSecond, double maxAccelMetersPerSecondSquare, 
      double maxPositionErrorMeters, double maxThetaErrorDegrees, 
      boolean closedLoopSwerve, boolean usePositionFeedback,
      DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.maxPositionErrorMeters = maxPositionErrorMeters;
    this.maxThetaErrorDegrees = maxThetaErrorDegrees;
    this.poseType = poseType;
    goalSupplier = goalPoseSupplier;
    goalMode = GoalMode.poseSupplier;
    this.openLoopSwerve = !closedLoopSwerve;

    trapProfileConstraints = new TrapezoidProfileBCR.Constraints(
      MathUtil.clamp(maxVelMetersPerSecond, -SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullSpeedMetersPerSecond), 
      MathUtil.clamp(maxAccelMetersPerSecondSquare, -SwerveConstants.kFullAccelerationMetersPerSecondSquare, SwerveConstants.kFullAccelerationMetersPerSecondSquare)
    );

    constructorCommonCode();

    // Contoller object is created in constructorCommonCode(), so setting the controller enabled/disabled has to occur afterwards
    controller.setEnabled(usePositionFeedback);
  }

  /**
   * Drives the robot to the desired pose in field-relative or robot-relative coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param poseType Determines how goalPose is interpreted:
   *  <p> kRelative = goalPose is in the robot coordinate system
   *  <p> kAbsolute = goalPose is in the Blue field coordinate system, based on the Blue alliance. The goal pose is unaffected by the currently selected alliance (red or blue).
   *  <p> kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use. Behaves same as kAbsolute.
   * @param goalPoseSupplier A function that supplies the target pose. Pose components include
   *  <p> For poseType = kRelative:
   *  <p> Robot forward movement, in meters (0 = current robot location, + = robot forward, - = robot reverse)
   *  <p> Robot left movement, in meters (0 = current robot location, + = robot left, - = robot right)
   *  <p> Robot angle, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   *  <p> For poseType = kAbsolute:
   *  <p> Desired X location in the field, in meters (0 = field edge in front of the Blue driver station, + = away from the Blue drivestation)
   *  <p> Desired Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, + = left when looking from the Blue drivestation)
   *  <p> Desired angle on the field, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   * @param driveTrain DriveTrain subsystem
   */
  public DriveToPose(CoordType poseType, Supplier<Pose2d> goalPoseSupplier, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.poseType = poseType;
    goalSupplier = goalPoseSupplier;
    goalMode = GoalMode.poseSupplier;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }


   /**
   * Drives the robot to the desired pose in field-relative or robot-relative coordinates.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param poseType Determines how goalPose is interpreted:
   *  <p> kRelative = goalPose is in the robot coordinate system
   *  <p> kAbsolute = goalPose is in the Blue field coordinate system, based on the Blue alliance. The goal pose is unaffected by the currently selected alliance (red or blue).
   *  <p> kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use. Behaves same as kAbsolute.
   * @param goalPoseSupplier A function that supplies the target pose. Pose components include
   *  <p> For poseType = kRelative:
   *  <p> Robot forward movement, in meters (0 = current robot location, + = robot forward, - = robot reverse)
   *  <p> Robot left movement, in meters (0 = current robot location, + = robot left, - = robot right)
   *  <p> Robot angle, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   *  <p> For poseType = kAbsolute:
   *  <p> Desired X location in the field, in meters (0 = field edge in front of the Blue driver station, + = away from the Blue drivestation)
   *  <p> Desired Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, + = left when looking from the Blue drivestation)
   *  <p> Desired angle on the field, in degrees (0 = facing away from the Blue drivestation, + to the left, - to the right)
   * @param maxPositionErrorMeters tolerance for end position in meters
   * @param maxThetaErrorDegrees tolerance for end theta in degrees
   * @param driveTrain DriveTrain subsystem
   */
  public DriveToPose(CoordType poseType, Supplier<Pose2d> goalPoseSupplier, double maxPositionErrorMeters, double maxThetaErrorDegrees, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.poseType = poseType;
    this.maxPositionErrorMeters = maxPositionErrorMeters;
    this.maxThetaErrorDegrees = maxThetaErrorDegrees;
    goalSupplier = goalPoseSupplier;
    goalMode = GoalMode.poseSupplier;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }


  /**
   * Rotates the robot to the specified rotation using an arbitrary angle without moving laterally.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param type CoordType, kRelative (turn relative to current robot angle) or kAbsolute (turn to Blue field angle). kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use (behaves same as kAbsolute).
   * @param rotation rotation to turn to, in degrees (+=turn left, -=turn right). For absolute rotation,
   * the 0 degrees is facing away from the Blue driver station.
   * @param driveTrain DriveTrain subsytem
   */
  public DriveToPose(CoordType type, double rotation, DriveTrain driveTrain){
    this.driveTrain = driveTrain;
    
    this.rotation = Rotation2d.fromDegrees(rotation);
    this.poseType = type;

    if (type == CoordType.kRelative) {
      goalMode = GoalMode.angleRelative;
    } else {
      goalMode = GoalMode.angleAbsolute;
    }

    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();
  }

  /**
   * Drives the robot to the desired pose based on numbers inputed in shuffleboard.
   * Stops the robot at the end of the command, unless the command is interrupted.
   * @param poseType Determines how goalPose is interpreted:
   *  <p> kRelative = goalPose is in the robot coordinate system
   *  <p> kAbsolute = goalPose is in the Blue field coordinate system, based on the Blue alliance. The goal pose is unaffected by the currently selected alliance (red or blue).
   *  <p> kAbsoluteResetPose, kAbsoluteResetPoseTol = Don't use. Behaves same as kAbsolute.
   * @param driveTrain DriveTrain subsystem
   */
  public DriveToPose(CoordType poseType, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.poseType = poseType;
    goalMode = GoalMode.shuffleboard;
    trapProfileConstraints = TrajectoryConstants.kDriveProfileConstraints;

    constructorCommonCode();

    if (SmartDashboard.getNumber("DriveToPose XPos meters", -9999) == -9999) {
      SmartDashboard.putNumber("DriveToPose XPos meters", 2);
    }
    if (SmartDashboard.getNumber("DriveToPose YPos meters", -9999) == -9999){
      SmartDashboard.putNumber("DriveToPose YPos meters", 2);
    }
    if (SmartDashboard.getNumber("DriveToPose Rot degrees", -9999) == -9999) {
      SmartDashboard.putNumber("DriveToPose Rot degrees", 0);
    }
    if (SmartDashboard.getNumber("DriveToPose VelMax mps", -9999) == -9999) {
      SmartDashboard.putNumber("DriveToPose VelMax mps", 2);
    }

    // if(feedbackChooser.getSelected() == null) {
      feedbackChooser.setDefaultOption("Normal", FEEDBACK_NORMAL);
      feedbackChooser.addOption("Velocity only", FEEDBACK_VELOCITY_ONLY);
      feedbackChooser.addOption("None", FEEDBACK_NONE);
      SmartDashboard.putData("DriveToPose Feedback", feedbackChooser);
    // }
  }

  /**
   * Common code between multiple constructors.
   */
  private void constructorCommonCode() {
    addRequirements(driveTrain);

    // Define the controller for robot rotation
    ProfiledPIDController thetaController = new ProfiledPIDController(
        TrajectoryConstants.kPThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Define the holomic controller to controll driving
    controller = new HolonomicDriveControllerBCR(
      new PIDController(TrajectoryConstants.kPXController, 0, 0),
      new PIDController(TrajectoryConstants.kPYController, 0, 0),
      thetaController );

    // Prime logging
    long timeNow = RobotController.getFPGATime();
    Pose2d robotPose = driveTrain.getPose();
    ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();

    dLogTime.append(-1, timeNow);
    dLogCurPose2D.append(robotPose, timeNow);
    dLogTrajPose2D.append(robotPose, timeNow);
    dLogTrajX.append(-1, timeNow);
    dLogTrajY.append(-1, timeNow);
    dLogTrajAccel.append(-1, timeNow);
    dLogTrajVel.append(-1, timeNow);
    dLogTrajVelkA.append(-1, timeNow);
    dLogTrajVelAng.append(-1, timeNow);
    dLogTrajRot.append(-1, timeNow);
    dLogRobotPosErr.append(-1, timeNow);
    dLogRobotThErr.append(-1, timeNow);
    dLogRobotX.append(robotPose.getX(), timeNow);
    dLogRobotY.append(robotPose.getY(), timeNow);
    dLogRobotVel.append(Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond), timeNow);
    dLogRobotXVel.append(robotSpeeds.vxMetersPerSecond, timeNow);
    dLogRobotYVel.append(robotSpeeds.vyMetersPerSecond, timeNow);
    dLogRobotVelAng.append(Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)), timeNow);
    dLogRobotRot.append(robotPose.getRotation().getDegrees(), timeNow);
    dLogRobotPitch.append(driveTrain.getGyroPitch(), timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset timer and controllers
    timer.reset();
    timer.start();
    controller.reset();
    controller.setEnabled(true);

    // Get the initial pose
    initialPose = driveTrain.getPose();
    initialTranslation = initialPose.getTranslation();
    curRobotTranslation = initialTranslation;

    // Get the goal pose
    switch (goalMode) {
      // Goal pose directly specified
      case pose:
        break;
      // Using a supplier in the constructor
      case poseSupplier:
        inputPose = goalSupplier.get();
        break;
      // Using Shuffleboard
      case shuffleboard:
        double xPos = SmartDashboard.getNumber("DriveToPose XPos meters", 0);
        double yPos = SmartDashboard.getNumber("DriveToPose YPos meters", 0);
        Rotation2d angleTarget = Rotation2d.fromDegrees(SmartDashboard.getNumber("DriveToPose Rot degrees", 0));
        inputPose = new Pose2d(xPos, yPos, angleTarget);

        trapProfileConstraints.maxVelocity = MathUtil.clamp(SmartDashboard.getNumber("DriveToPose VelMax mps", 0), 
          -SwerveConstants.kFullSpeedMetersPerSecond, SwerveConstants.kFullSpeedMetersPerSecond);

        int feedbackMode = feedbackChooser.getSelected();
        switch (feedbackMode) {
          case FEEDBACK_NONE:
            openLoopSwerve = true;
            controller.setEnabled(false);
            break;
          case FEEDBACK_VELOCITY_ONLY:
            openLoopSwerve = false;
            controller.setEnabled(false);
            break;
          default:  // Normal driving
            openLoopSwerve = false;
            controller.setEnabled(true);
            break;
        }
        break;
      // Absolute angle, keep robot position
      case angleAbsolute:
        goalPose = new Pose2d(driveTrain.getPose().getTranslation(), rotation);
        break;
      // Relative angle, keep robot position
      case angleRelative:
        goalPose = driveTrain.getPose().plus(new Transform2d(new Translation2d(), rotation));
        break;
    }

    // Adjust pose for relative mode, if it was selected
    if (goalMode == GoalMode.pose || goalMode == GoalMode.poseSupplier || goalMode == GoalMode.shuffleboard) {
      if (poseType == CoordType.kRelative) {
        goalPose = driveTrain.getPose().plus((new Transform2d(inputPose.getX(), inputPose.getY(), inputPose.getRotation())));
      } else {
        goalPose = inputPose;
      }

      DataLogUtil.writeMessage("DriveToPose: Init");
      dLogTrajType.append("DriveToPose");

    }

    // Calculate the direction and distance of travel
    Translation2d trapezoidPath = goalPose.getTranslation().minus(initialTranslation);
    goalDirection = Translation2dBCR.normalize(trapezoidPath);
    double goalDistance = trapezoidPath.getNorm();
    
    // Get the initial velocity in the direction of travel
    ChassisSpeeds robotSpeed = driveTrain.getRobotSpeeds();
    double initialVelocity = robotSpeed.vxMetersPerSecond * goalDirection.getX() + robotSpeed.vyMetersPerSecond * goalDirection.getY();

    // Create the profile, which is linear distance (along goalDirection) relative to the initial pose
    TrapezoidProfileBCR.State initialState = new TrapezoidProfileBCR.State(0, initialVelocity);
    TrapezoidProfileBCR.State goalState = new TrapezoidProfileBCR.State(goalDistance, 0);
    profile = new TrapezoidProfileBCR(trapProfileConstraints, goalState, initialState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();

    // Current robot location, translation is relative to starting position, rotation is absolute field rotation
    Pose2d curRobotOdometryPose = driveTrain.getPose();
    curRobotTranslation = curRobotOdometryPose.getTranslation().minus(initialTranslation);
    Pose2d robotPose = new Pose2d(curRobotTranslation, Rotation2d.fromDegrees(driveTrain.getPoseAngle()));

    // Calculate current desired pose and velocity from the Trapezoid profile, relative to starting position
    TrapezoidProfileBCR.State desiredState = profile.calculate(curTime);
    Pose2d desiredPose = new Pose2d( goalDirection.times(desiredState.position), goalDirection.getAngle());

    // Fudge in some kA
    double desiredVelocityMetersPerSecond = desiredState.velocity + (desiredState.acceleration * SwerveConstants.kADriveToPose);

    Rotation2d desiredRotation = goalPose.getRotation();

    // Get the field-relative chassis speeds
    ChassisSpeeds targetChassisSpeeds = controller.calculate(robotPose, desiredPose, desiredVelocityMetersPerSecond, desiredRotation);

    driveTrain.drive(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond,
        targetChassisSpeeds.omegaRadiansPerSecond, DriveType.kFieldRelative, openLoopSwerve);

    // Log data for path following
    long timeNow = RobotController.getFPGATime();
    ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
    dLogTime.append(curTime, timeNow);
    dLogCurPose2D.append(curRobotOdometryPose, timeNow);
    dLogTrajPose2D.append(new Pose2d(desiredPose.getTranslation().plus(initialTranslation), desiredRotation), timeNow);   // Field-relative current desired position
    dLogTrajX.append(desiredPose.getTranslation().getX(), timeNow);
    dLogTrajY.append(desiredPose.getTranslation().getY(), timeNow);
    dLogTrajAccel.append(desiredState.acceleration, timeNow);
    dLogTrajVel.append(desiredState.velocity, timeNow);
    dLogTrajVelkA.append(desiredVelocityMetersPerSecond, timeNow);
    dLogTrajVelAng.append(desiredPose.getRotation().getDegrees(), timeNow);
    dLogTrajRot.append(desiredRotation.getDegrees(), timeNow);
    dLogRobotPosErr.append(driveTrain.getPose().getTranslation().minus(goalPose.getTranslation()).getNorm(), timeNow);
    dLogRobotThErr.append(MathBCR.angleMinus(driveTrain.getPoseAngle(), goalPose.getRotation().getDegrees()), timeNow);
    dLogRobotX.append(curRobotTranslation.getX(), timeNow);
    dLogRobotY.append(curRobotTranslation.getY(), timeNow);
    dLogRobotVel.append(Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond), timeNow);
    dLogRobotXVel.append(robotSpeeds.vxMetersPerSecond, timeNow);
    dLogRobotYVel.append(robotSpeeds.vyMetersPerSecond, timeNow);
    dLogRobotVelAng.append(Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)), timeNow);
    dLogRobotRot.append(robotPose.getRotation().getDegrees(), timeNow);
    dLogRobotPitch.append(driveTrain.getGyroPitch(), timeNow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (!interrupted) driveTrain.stopMotors();
    dLogTrajType.append("None");
    DataLogUtil.writeMessage(false, "DriveToPose: End, interrupted =", interrupted); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var timeout = timer.hasElapsed(profile.totalTime() + 3.0);
    if (timeout) DataLogUtil.writeMessage( "DriveToPose timed out"); 

    var angleError = MathBCR.angleMinus(driveTrain.getPoseAngle(), goalPose.getRotation().getDegrees());
    var posError = driveTrain.getPose().getTranslation().minus(goalPose.getTranslation()).getNorm();
    
    // If we are 3 seconds after the profile completed, then end even if we are not within tolerance 
    var finished = timeout ||
        (timer.hasElapsed(profile.totalTime()) &&
            (Math.abs(angleError) <= maxThetaErrorDegrees) &&
            (posError <= maxPositionErrorMeters));

    if (finished) {
      DataLogUtil.writeMessage("DriveToPose: Finished, angleError =", angleError, "posError", posError, 
                "maxTheta =", maxThetaErrorDegrees, "maxMeters =", maxPositionErrorMeters, "timer =", timer.get());
    }

    return finished;
  }
}
