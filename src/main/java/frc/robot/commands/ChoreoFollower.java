// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.StopType;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.MathBCR;

public class ChoreoFollower extends Command {
  private Timer timer = new Timer();
  private Trajectory<SwerveSample> trajectory;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Consumer<ChassisSpeeds> outputChassisSpeed;
  private Supplier<Pose2d> poseSupplier;
  private BooleanSupplier mirrorTrajectory;
  private boolean mirrorTrajectoryThisInit;
  private DriveTrain driveTrain;
  private final StopType stopType;

  private boolean noTrajectory;     // Trajectory has no points
  private Pose2d finalPose;         // Desired pose at end of trajectory
  private double finalVelocity;     // Desired velocity at end of trajectory
  
  private final DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> dLogCurPose2D = StructLogEntry.create(log, "/DrivePathFollower/curPose2d", Pose2d.struct);
  private final StructLogEntry<Pose2d> dLogTrajPose2D = StructLogEntry.create(log, "/DrivePathFollower/trajPose2d", Pose2d.struct);
  private final StringLogEntry dLogTrajType = new StringLogEntry(log, "/DrivePathFollower/trajType");
  private final DoubleLogEntry dLogTime = new DoubleLogEntry(log, "/DrivePathFollower/time");
  private final DoubleLogEntry dLogTrajX = new DoubleLogEntry(log, "/DrivePathFollower/trajX");
  private final DoubleLogEntry dLogTrajY = new DoubleLogEntry(log, "/DrivePathFollower/trajY");
  private final DoubleLogEntry dLogTrajVel = new DoubleLogEntry(log, "/DrivePathFollower/trajVel");
  private final DoubleLogEntry dLogTrajVelAng = new DoubleLogEntry(log, "/DrivePathFollower/trajVelAng");
  private final DoubleLogEntry dLogTrajRot = new DoubleLogEntry(log, "/DrivePathFollower/trajRot");
  private final DoubleLogEntry dLogTrajRotVel = new DoubleLogEntry(log, "/DrivePathFollower/trajRotVel");
  private final DoubleLogEntry dLogRobotX = new DoubleLogEntry(log, "/DrivePathFollower/robotX");
  private final DoubleLogEntry dLogRobotY = new DoubleLogEntry(log, "/DrivePathFollower/robotY");
  private final DoubleLogEntry dLogRobotVel = new DoubleLogEntry(log, "/DrivePathFollower/robotVel");
  private final DoubleLogEntry dLogRobotVelAng = new DoubleLogEntry(log, "/DrivePathFollower/robotVelAng");
  private final DoubleLogEntry dLogRobotRot = new DoubleLogEntry(log, "/DrivePathFollower/robotRot");
  private final DoubleLogEntry dLogRobotRotVel = new DoubleLogEntry(log, "/DrivePathFollower/robotRotVel");

  /**
   * Choreo follower used to follow Choreo trajectories. 
   * This method takes in a choreo trajectory and follows the contents of that trajectory until the trajectory is finished.
   * @param stopType Specify behavior at the end of the trajectory
   *  <p> kNoStop = Don't stop the drivetrain, don't reset Coast/Brake mode.
   *      Continue the drivetrain motors in their current state (wheel orientation and velocity) when the trajectory ends.  Note that
   *      this may not exactly match the end of the planned trajectory (just because the end of the trajectory is a stop point [zero velocity], that does not mean
   *      that the robot will actually be stopped when this command ends).
   *  <p>kCoast = Stop the drive motors and set the drivetrain to Coast mode
   *  <p>kBrake = Stop the drive motors and set the drivetrain to Brake mode
   * @param trajectory Choreo trajectory to follow
   * @param xController trajectory PID X feedback controller
   * @param yController trajectory PID Y feedback controller 
   * @param rotationController rotation PID feedback controller
   * @param outputChassisSpeeds a function that takes in a field-relative chassis speeds and controls the robot
   * @param poseSupplier a pose supplier to get the current robot position
   * @param mirrorTrajectory determines if the trajectory will be mirrored: when true the trajectory will mirror while 
   * internal robot odometry will remain the same, when false the trajectory will not be changed
   * @param driveTrain DriveTrain subsystem
   */
  public ChoreoFollower(StopType stopType, Trajectory<SwerveSample> trajectory, 
      PIDController xController, PIDController yController,
      PIDController rotationController, Consumer<ChassisSpeeds> outputChassisSpeeds,
      Supplier<Pose2d> poseSupplier, BooleanSupplier mirrorTrajectory, DriveTrain driveTrain) {
    this.stopType = stopType;
    this.trajectory = trajectory;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;
    this.outputChassisSpeed = outputChassisSpeeds;
    this.poseSupplier = poseSupplier;
    this.mirrorTrajectory = mirrorTrajectory;
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);

    // Prime data logging at boot time
    long timeNow = RobotController.getFPGATime();
    Pose2d poseCurr = new Pose2d();  //2025 code -> poseSupplier.get();  TODO using poseSupplier.get() instead of new Pose2d() causes crashes in 2026 code.  Why?
    ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
    dLogCurPose2D.append(poseCurr, timeNow);
    dLogTrajPose2D.append(poseCurr, timeNow);

    dLogTime.append(-1, timeNow);
    dLogTrajX.append(-1, timeNow);
    dLogTrajY.append(-1, timeNow);
    dLogTrajVel.append(-1, timeNow);
    dLogTrajVelAng.append(-1, timeNow);
    dLogTrajRot.append(-1, timeNow);
    dLogTrajRotVel.append(-1, timeNow);
    dLogRobotX.append(poseCurr.getX(), timeNow);
    dLogRobotY.append(poseCurr.getY(), timeNow);
    dLogRobotVel.append(Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond), timeNow);
    dLogRobotVelAng.append(Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)), timeNow);
    dLogRobotRot.append(poseCurr.getRotation().getDegrees(), timeNow);
    dLogRobotRotVel.append(driveTrain.getAngularVelocity(), timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrain.enableFastLogging(true);
    
    timer.reset();
    timer.start();

    xController.reset();
    yController.reset();
    rotationController.reset();

    mirrorTrajectoryThisInit = mirrorTrajectory.getAsBoolean();

    Optional<SwerveSample> finalSampleOpt = trajectory.getFinalSample(mirrorTrajectoryThisInit);
    noTrajectory = finalSampleOpt.isEmpty();
    if (!noTrajectory) {
      finalPose = finalSampleOpt.get().getPose();
      finalVelocity = Math.hypot( finalSampleOpt.get().getChassisSpeeds().vxMetersPerSecond,
        finalSampleOpt.get().getChassisSpeeds().vyMetersPerSecond);
    }

    DataLogUtil.writeMessage("ChoreoFollower:  Init, Is Flipped =", mirrorTrajectoryThisInit, ", Stop Type =", stopType, ", Trajectory =", trajectory.name());
    dLogTrajType.append("ChoreoFollower");
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xFF;
    double yFF;
    double rotationFF;
    double xFeedback ;
    double yFeedback;
    double rotationFeedback;

    double curTime = timer.get();

    Optional<SwerveSample> sampleCurrOpt = trajectory.sampleAt(curTime, mirrorTrajectoryThisInit);
    Optional<SwerveSample> sampleNextOpt = trajectory.sampleAt(curTime + Constants.SwerveConstants.dt, mirrorTrajectoryThisInit);

    if(!sampleCurrOpt.isEmpty() && !sampleNextOpt.isEmpty()){
      SwerveSample sampleCurr = sampleCurrOpt.get();
      SwerveSample sampleNext = sampleNextOpt.get();
      Pose2d poseCurr = poseSupplier.get();

      xFF = sampleNext.vx;
      yFF = sampleNext.vy;
      rotationFF = sampleNext.omega;
      xFeedback = xController.calculate(poseCurr.getX(), sampleCurr.x);
      yFeedback = yController.calculate(poseCurr.getY(), sampleCurr.y);
      rotationFeedback = rotationController.calculate(poseCurr.getRotation().getRadians(), sampleCurr.heading);

      // Log data for trajectory following
      long timeNow = RobotController.getFPGATime();
      ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
      dLogCurPose2D.append(poseCurr, timeNow);
      dLogTrajPose2D.append(sampleCurr.getPose(), timeNow);

      dLogTime.append(curTime, timeNow);
      dLogTrajX.append(sampleCurr.x, timeNow);
      dLogTrajY.append(sampleCurr.y, timeNow);
      dLogTrajVel.append(Math.hypot(sampleCurr.vx, sampleCurr.vy), timeNow);
      dLogTrajVelAng.append(Math.toDegrees(Math.atan2(sampleCurr.vy, sampleCurr.vx)), timeNow);
      dLogTrajRot.append(Math.toDegrees(sampleCurr.heading), timeNow);
      dLogTrajRotVel.append(Math.toDegrees(sampleCurr.omega), timeNow);
      dLogRobotX.append(poseCurr.getX(), timeNow);
      dLogRobotY.append(poseCurr.getY(), timeNow);
      dLogRobotVel.append(Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond), timeNow);
      dLogRobotVelAng.append(Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)), timeNow);
      dLogRobotRot.append(poseCurr.getRotation().getDegrees(), timeNow);
      dLogRobotRotVel.append(driveTrain.getAngularVelocity(), timeNow);

      // Log data from swereve modules
      driveTrain.updateDriveLogModuleOutputs(timeNow);
        
    } else {
      xFF = 0;
      yFF = 0;
      rotationFF = 0;
      xFeedback = 0;
      yFeedback = 0;
      rotationFeedback = 0;
    }

    outputChassisSpeed.accept(new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrain.enableFastLogging(false);
    timer.stop();

    // Add any final behavior, per the stopAtEnd
    if (stopType == StopType.kBrake) {
      driveTrain.stopMotors();
      driveTrain.setDriveModeCoast(false);
    } else if (stopType == StopType.kCoast) {
      driveTrain.stopMotors();
      driveTrain.setDriveModeCoast(true);
    }

    DataLogUtil.writeMessage("ChoreoFollower:  End, interrupted =", interrupted);
    dLogTrajType.append("None");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d poseCurr = poseSupplier.get();

    return (
        // Stop if there is no trajectory
        noTrajectory
        ||
        // Stop if we are past the end of the trajectory *and* the PID controllers have pulled
        // the robot within tolerance of the final point of the trajctory
        timer.hasElapsed(trajectory.getTotalTime())
        && Math.abs(finalPose.getX() - poseCurr.getX()) <= Constants.TrajectoryConstants.xError
        && Math.abs(finalPose.getY() - poseCurr.getY()) <= Constants.TrajectoryConstants.yError
        && Math.abs(MathBCR.normalizeAngle(poseCurr.getRotation().getDegrees()
            - finalPose.getRotation().getDegrees())) <= Constants.TrajectoryConstants.choreoMaxThetaErrorDegrees
        && Math.hypot(driveTrain.getChassisSpeeds().vxMetersPerSecond,
            driveTrain.getChassisSpeeds().vyMetersPerSecond) <= Constants.TrajectoryConstants.endVelocityError)
        ||
        // If final point of the trajctory has a non-zero velocity,
        // then stop immediately at the end of the trajectory
        timer.hasElapsed(trajectory.getTotalTime())
        && finalVelocity > 0.01
        ||
        // Stop if we are more than 0.7sec past the end of the trajectory
        timer.hasElapsed(trajectory.getTotalTime() + 0.7);
  }
}
