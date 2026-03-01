/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class DriveStraight extends Command {
  private DriveTrain driveTrain;
  private boolean fieldRelative;
  private double target;                  // How many more degrees to the right to turn
  private double maxVel;                  // Max velocity, between 0 and SwerveConstants.DrivekMaxSpeedMetersPerSecond 
  private double maxAccel;                // Max acceleration, between 0 and SwerveConstants.kMaxAccelerationMetersPerSecondSquared
  private long profileStartTime;          // Initial time (time of starting point)
  private double currDist;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private double angleInput, angleTarget; // angleTarget is relative to the starting robot facing
  private Translation2d startLocation;
  private SwerveModuleState[] desiredStates;
  
  private boolean isOpenLoop;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile;                 // WPILib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr;         // Initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext;         // Next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal;        // Goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // Max vel (deg/sec) and max accel (deg/sec/sec) of the system
 
  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTargetAngle = new DoubleLogEntry(log, "/DriveStraight/targetAng");
  private final DoubleLogEntry dLogTargetPos = new DoubleLogEntry(log, "/DriveStraight/targetPos");
  private final DoubleLogEntry dLogTargetVel = new DoubleLogEntry(log, "/DriveStraight/targetVel");
  private final DoubleLogEntry dLogTargetAccel = new DoubleLogEntry(log, "/DriveStraight/targetAccel");
  private final DoubleLogEntry dLogRobotPos = new DoubleLogEntry(log, "/DriveStraight/robotPos");
  private final DoubleLogEntry dLogRobotVel = new DoubleLogEntry(log, "/DriveStraight/robotVel");
  private final DoubleLogEntry dLogRobotVelFL = new DoubleLogEntry(log, "/DriveStraight/robotVelFL");
  private final DoubleLogEntry dLogRobotVelFR = new DoubleLogEntry(log, "/DriveStraight/robotVelFR");
  private final DoubleLogEntry dLogRobotVelBL = new DoubleLogEntry(log, "/DriveStraight/robotVelBL");
  private final DoubleLogEntry dLogRobotVelBR = new DoubleLogEntry(log, "/DriveStraight/robotVelBR");

  /**
   * Drives the robot straight.
   * <p><b>FOR CALIBRATION ONLY.</b> Use DriveToPose for competition code.
   * @param target distance to travel, in meters (positive = forward, negative = backward)
   * @param fieldRelative false = angle is relative to current robot facing, true = angle is an absolute field angle (0 = away from Blue driver station)
   * @param angle angle to drive along when driving straight, in degrees (positive = left, negative = right)
   * @param maxVel max velocity in meters/second, between 0 and kMaxSpeedMetersPerSecond in Constants
   * @param maxAccel max acceleration in meters/second2, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = do not regenerate (for debugging)
   * @param isOpenLoop true = feed-forward only for velocity control, false = PID feedback velocity control
   * @param driveTrain DriveTrain subsystem
   */
  public DriveStraight(double target, boolean fieldRelative, double angle, double maxVel, double maxAccel, boolean regenerate, boolean isOpenLoop, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.fieldRelative = fieldRelative;
    angleInput = angle;
    this.regenerate = regenerate;
    this.isOpenLoop = isOpenLoop;
    this.fromShuffleboard = false;
    this.target = target;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, SwerveConstants.kMaxSpeedMetersPerSecond);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, SwerveConstants.kMaxAccelerationMetersPerSecondSquare);

    addRequirements(driveTrain);
    initializeDataLog();
  }

  /**
   * Drives the robot straight, using values from Shuffleboard.
   * <p><b>FOR CALIBRATION ONLY.</b> Use DriveToPose for competition code.
   * @param fieldRelative false = angle is relative to current robot facing, true = angle is an absolute field angle (0 = away from Blue driver station)
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = do not regenerate (for debugging)
   * @param isOpenLoop true = feed-forward only for velocity control, false = PID feedback velocity control
   * @param driveTrain DriveTrain subsystem
   */
  public DriveStraight(boolean fieldRelative, boolean regenerate, boolean isOpenLoop, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.fieldRelative = fieldRelative;
    angleInput = 0;
    this.regenerate = regenerate;
    this.isOpenLoop = isOpenLoop;
    this.fromShuffleboard = true;
    this.target = 0;
    this.maxVel = 0.5 * SwerveConstants.kMaxSpeedMetersPerSecond;
    this.maxAccel = 0.5 * SwerveConstants.kMaxAccelerationMetersPerSecondSquare;

    addRequirements(driveTrain);
    initializeDataLog();

    if (SmartDashboard.getNumber("DriveStraight Manual Target Dist", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Target Dist", 2);
    }
    if (SmartDashboard.getNumber("DriveStraight Manual Angle", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Angle", 0);
    }
    if (SmartDashboard.getNumber("DriveStraight Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxVel", 0.5 * SwerveConstants.kMaxSpeedMetersPerSecond);
    }
    if (SmartDashboard.getNumber("DriveStraight Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxAccel", 0.5 * SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
    }
  }

  /**
   * Initialize datalog when constructing this object, in order to prime the log at boot time.
   */
  public void initializeDataLog() {
    // Prime data logging at boot time
    long timeNow = RobotController.getFPGATime();
    dLogTargetAngle.append(-1, timeNow);
    dLogTargetPos.append(-1, timeNow);
    dLogTargetVel.append(-1, timeNow);
    dLogTargetAccel.append(-1, timeNow);
    dLogRobotPos.append(-1, timeNow);
    dLogRobotVel.append(-1, timeNow);
    dLogRobotVelFL.append(-1, timeNow);
    dLogRobotVelFR.append(-1, timeNow);
    dLogRobotVelBL.append(-1, timeNow);
    dLogRobotVelBR.append(-1, timeNow);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      target = SmartDashboard.getNumber("DriveStraight Manual Target Dist", 2);
      angleInput = SmartDashboard.getNumber("DriveStraight Manual Angle", 0);
      maxVel = SmartDashboard.getNumber("DriveStraight Manual MaxVel", SwerveConstants.kMaxSpeedMetersPerSecond);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, SwerveConstants.kMaxSpeedMetersPerSecond);
      maxAccel = SmartDashboard.getNumber("DriveStraight Manual MaxAccel", SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, SwerveConstants.kMaxAccelerationMetersPerSecondSquare);
    }

    // Calculate target angle
    if (fieldRelative) {
      angleTarget = angleInput - driveTrain.getPoseAngle();
    } else {
      angleTarget = angleInput;
    }

    // Initialize swerve states
    SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(angleTarget));
    desiredStates = new SwerveModuleState[] { state, state, state, state };

    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0);          // Initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0);     // Initialize initial state (relative turning, so assume initPos is 0 degrees)
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel);       // Initialize velocity and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);  // Generate profile
    DataLogUtil.writeMessage("DriveStraight: Init, Target =", target, ", Profile total time =", tProfile.totalTime());
    
    profileStartTime = System.currentTimeMillis();
    startLocation = driveTrain.getPose().getTranslation();
    
    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update data for this iteration
    long currProfileTime = System.currentTimeMillis();
    double timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    currDist = driveTrain.getPose().getTranslation().getDistance(startLocation);

    // Get next state from trapezoid profile
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);
    double targetVel = tStateNext.velocity;
    double targetAccel = tStateNext.acceleration;

    // Set wheel speeds
    // NOTE: All 4 SwerveModuleStates in the desiredStates[] array point to the same SwerveModuleState object.
    // So, they will always have the same speed, even if we only update one of the elements of the array.
    desiredStates[0].speedMetersPerSecond = targetVel;
    desiredStates[1].speedMetersPerSecond = targetVel;
    desiredStates[2].speedMetersPerSecond = targetVel;
    desiredStates[3].speedMetersPerSecond = targetVel;
    driveTrain.setModuleStates(desiredStates, isOpenLoop); 
    
    // Read current module states for logging
    SwerveModuleState[] currentStates = driveTrain.getModuleStates();
    double linearVel = (Math.abs(currentStates[0].speedMetersPerSecond)
        + Math.abs(currentStates[1].speedMetersPerSecond) + Math.abs(currentStates[2].speedMetersPerSecond)
        + Math.abs(currentStates[3].speedMetersPerSecond)) / 4.0;

    // Log data
    long timeNow = RobotController.getFPGATime();
    dLogTargetAngle.append(angleTarget, timeNow);
    dLogTargetPos.append(tStateNext.position, timeNow);
    dLogTargetVel.append(targetVel, timeNow);
    dLogTargetAccel.append(targetAccel, timeNow);
    dLogRobotPos.append(currDist, timeNow);
    dLogRobotVel.append(linearVel, timeNow);
    dLogRobotVelFL.append(currentStates[0].speedMetersPerSecond, timeNow);
    dLogRobotVelFR.append(currentStates[1].speedMetersPerSecond, timeNow);
    dLogRobotVelBL.append(currentStates[2].speedMetersPerSecond, timeNow);
    dLogRobotVelBR.append(currentStates[3].speedMetersPerSecond, timeNow);

    if (regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currDist, linearVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("DriveStraight: End");
    driveTrain.stopMotors();
    driveTrain.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(target - currDist) < 0.0125) {
      accuracyCounter++;
      DataLogUtil.writeMessage("DriveStraight: WithinTolerance, Target Dist =", target, ", Actual Dist =", currDist, ", Counter =", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}