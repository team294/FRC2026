// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveType;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

import java.lang.Math;

public class DriveWithJoysticksAdvanced extends Command {
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final AllianceSelection allianceSelection;
  private final DriveTrain driveTrain;
  
  private ProfiledPIDController turnRateController;
  private boolean firstInDeadband, firstCorrecting;
  private boolean stopped;                    // Joysticks are all in deadband, and robot has turned to the desired heading. Stop the robot from moving/jittering until the driver moves a joystick.
  private int logRotationKey;
  private double fwdVelocity, leftVelocity, turnRate, nextTurnRate;
  private double goalAngle;                   // In radians
  private double startTime;

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogFwdVelo = new DoubleLogEntry(log, "/DriveWithJoysticksAdvanced/Forward Velocity");
  private final DoubleLogEntry dLogLeftVelo = new DoubleLogEntry(log, "/DriveWithJoysticksAdvanced/Left Velocity");
  private final DoubleLogEntry dLogNextTurnRate = new DoubleLogEntry(log, "/DriveWithJoysticksAdvanced/Next Turn Rate");
  private final DoubleLogEntry dLogRobotAngle = new DoubleLogEntry(log, "/DriveWithJoysticksAdvanced/Robot Angle");
  private final DoubleLogEntry dLogRobotGoalAngle = new DoubleLogEntry(log, "/DriveWithJoysticksAdvanced/Robot Goal Angle");
  private final BooleanLogEntry dLogRobotStopped = new BooleanLogEntry(log, "/DriveWithJoysticksAdvanced/Stopped");

  /**
   * Control the driveTrain with joysticks using arcade drive and advanced controls.
   * @param leftJoystick left joystick, X and Y axis control robot movement, relative to the field from the perspective of the current Alliance's driver station
   * @param rightJoystick right joystick, X-axis controls robot rotation.
   * @param allianceSelection AllianceSelection utility
   * @param driveTrain DriveTrain subsystem
   */
  public DriveWithJoysticksAdvanced(Joystick leftJoystick, Joystick rightJoystick, AllianceSelection allianceSelection, DriveTrain driveTrain) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.allianceSelection = allianceSelection;
    this.driveTrain = driveTrain;

    turnRateController = new ProfiledPIDController(DriveConstants.kPJoystickThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    turnRateController.enableContinuousInput(-Math.PI, Math.PI);

    logRotationKey = DataLogUtil.allocateLogRotation();

    addRequirements(driveTrain);

    // Prime DataLog at boot time
    long timeNow = RobotController.getFPGATime();
    dLogFwdVelo.append(-1, timeNow);
    dLogLeftVelo.append(-1, timeNow);
    dLogNextTurnRate.append(-1, timeNow);
    dLogRobotAngle.append(-1, timeNow);
    dLogRobotGoalAngle.append(-1, timeNow);
    dLogRobotStopped.append(true, timeNow);
  }

  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    goalAngle = MathUtil.angleModulus(driveTrain.getPose().getRotation().getRadians());
    turnRateController.reset(goalAngle);    // Set the current setpoint for the controller
    turnRateController.setGoal(goalAngle);  // Set the goal for the controller

    if (Math.abs(driveTrain.getAngularVelocity()) < 0.5) {
      // Robot is rotating < 0.5 deg/sec. Assume the robot is not rotating, so use the current robot theta as the goal angle
      firstInDeadband = false;
      firstCorrecting = false;
      startTime = System.currentTimeMillis() - 200;
    } else {
      // The robot is still rotating. Wait 100ms prior to selecting the new goal angle
      firstInDeadband = true;
      firstCorrecting = true;
    }

    stopped = true; // Do not jitter when the robot is supposed to be still

    DataLogUtil.writeMessage("DriveWithJoysticksAdvanced:  Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fwdVelocity = (driveTrain.isXLocked()) ? 0.0 : (allianceSelection.getAlliance() == Alliance.Blue) ? -leftJoystick.getY() : leftJoystick.getY();
    leftVelocity = (driveTrain.isYLocked()) ? 0.0 : (allianceSelection.getAlliance() == Alliance.Blue) ? -leftJoystick.getX() : leftJoystick.getX();
    turnRate = (driveTrain.isTurnLocked()) ? 0.0 : -rightJoystick.getX();

    if (driveTrain.isFineControlEnabled()) {
      fwdVelocity *= DriveConstants.fineControlRatio;
      leftVelocity *= DriveConstants.fineControlRatio;
      turnRate *= DriveConstants.fineControlRatio;
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNone)) {
      SmartDashboard.putNumber("Joystick Left-Y", fwdVelocity);
      SmartDashboard.putNumber("Joystick Left-X", leftVelocity);
      SmartDashboard.putNumber("Joystick Right-X", turnRate);
    }

    // Apply deadbands and limits for fine control
    fwdVelocity = (Math.abs(fwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(fwdVelocity) * (SwerveConstants.kMaxSpeedMetersPerSecond);
    leftVelocity = (Math.abs(leftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(leftVelocity) * (SwerveConstants.kMaxSpeedMetersPerSecond);
    turnRate = (Math.abs(turnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(turnRate) * (SwerveConstants.kMaxTurningRadiansPerSecond);

    // If the driver moves any joystick, then turn off the "stopped" feature that prevents jittering when the robot is supposed to be sitting still.
    if (stopped && (fwdVelocity != 0.0 || leftVelocity != 0.0 || turnRate != 0.0)) {
      stopped = false;
      firstCorrecting = true; // Set to true to reset the goal angle before starting correcting (accounts for gyro drift, odometry updates, physically moving the robot, etc)
    }

    if (turnRate == 0) {
      // Use angle controller system if the theta joystick is in the deadband
      if (firstInDeadband) {
        firstInDeadband = false;
        driveTrain.enableFastLogging(true);

        // Start 100ms timer
        startTime = System.currentTimeMillis();
      }

      if (System.currentTimeMillis() - startTime > 100) {
        // Uses profiled PID controller for angle only if the theta joystick is in the deadband for more than 100ms.
        // The 100ms delay is the theta settling time to account for the robot's rotational inertia after the theta joystick is released.
  
        double curRobotAngle = MathUtil.angleModulus(driveTrain.getPose().getRotation().getRadians());  // Angle is in radians

        if (firstCorrecting) {
          firstCorrecting = false;
          driveTrain.enableFastLogging(false);

          // After 100ms, set the goalAngle for the robot theta controller
          goalAngle = curRobotAngle;
          // Set the current setpoint for the controller
          turnRateController.reset(goalAngle);
        }

        double angleError = MathUtil.angleModulus(goalAngle - curRobotAngle);
        boolean angleInTolerance = Math.abs(angleError) < Math.PI / 180.0;  // Calculate if robot angle is within 1 degree of goalAngle

        // Calculates using the profiledPIDController what the next turnRate should be
        if (!angleInTolerance && !stopped) {
          nextTurnRate = turnRateController.calculate(curRobotAngle, goalAngle);
        } else {
          nextTurnRate = 0;
          if (fwdVelocity == 0.0 && leftVelocity == 0.0) {
            stopped = true;
          }
        }

        if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNone)) {
          SmartDashboard.putNumber("DriveWJAdv Goal Angle", Math.toDegrees(goalAngle));
          SmartDashboard.putNumber("DriveWJAdv Robot Angle", Math.toDegrees(curRobotAngle));
          SmartDashboard.putNumber("DriveWJAdv Angle Error", Math.toDegrees(angleError));
          SmartDashboard.putBoolean("DriveWJAdv In Tolerance", angleInTolerance);
        }
        turnRate = nextTurnRate;
      } else {
        // Uses the regular turnRate if the theta joystick is in the deadband less than 100ms
        goalAngle = driveTrain.getPose().getRotation().getRadians();
      }

    } else {
      // Uses the regular turnRate if the theta joystick is not in the deadband
      if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNone)) {
        goalAngle = driveTrain.getPose().getRotation().getRadians();
      }
      
      firstInDeadband = true;
      firstCorrecting = true;
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNone)) {
      long timeNow = RobotController.getFPGATime();
      dLogFwdVelo.append(fwdVelocity, timeNow);
      dLogLeftVelo.append(leftVelocity, timeNow);
      dLogNextTurnRate.append(turnRate, timeNow);
      dLogRobotAngle.append(Math.toDegrees(driveTrain.getPose().getRotation().getRadians()), timeNow);
      dLogRobotGoalAngle.append(Math.toDegrees(goalAngle), timeNow);
      dLogRobotStopped.append(stopped, timeNow);
    }

    driveTrain.drive(fwdVelocity, leftVelocity, turnRate, DriveType.kDriverRelative, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("DriveWithJoysticksAdvanced:  End, interrupted =", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for linear travel.
   * @param rawJoystick raw joystick value, -1.0 to +1.0
   * @return scaled joystick value, -1.0 to +1.0
   */
  private double scaleTurn(double rawJoystick){
    return Math.signum(rawJoystick) * (0.6801 * rawJoystick * rawJoystick + 0.3232 * Math.abs(rawJoystick) - 0.0033);
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for rotating the robot.
   * @param rawJoystick raw joystick value, -1.0 to +1.0
   * @return scaled joystick value, -1.0 to +1.0
   */
  private double scaleJoystick(double rawJoystick){
    return Math.signum(rawJoystick) * (0.7912 * rawJoystick * rawJoystick + 0.2109 * Math.abs(rawJoystick) - 0.0022);
  }
}
