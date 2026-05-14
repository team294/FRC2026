// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElasticKeyConstants.ElasticKey;
import frc.robot.Constants.EndType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Field;
import frc.robot.utilities.LEDEventUtil;
import frc.robot.utilities.LEDEventUtil.LEDStripEvents;

public class ShooterScorePrep extends Command {
  private final Shooter shooter;
  private final Turret turret;
  private final DriveTrain driveTrain;
  private final Field field;
  private DoubleSupplier velocityGetter;
  private DoubleSupplier angleGetter;
  private Translation2d goalTranslation2d;
  private double targetVelocity;
  private double targetHoodAngle;
  private boolean useGetter = false;
  private boolean selfCalculate = false;
  private boolean fromElastic = false;

  private int toleranceCounter = 0;
  private double distanceToTarget;


  private final EndType type;

  private boolean ledSentShooterReadyEvent;

  /**
   * Sets the hood angle and shooter velocity for scoring.
   * @param velocityGetter DoubleSupplier that returns the target velocity of the shooter, in rpm
   * @param angleGetter DoubleSupplier that returns the target angle of the hood, in degrees
   * @param type EndType; don't use immediatelyEnd for the hood
   * <p> waitForSetPoint = command runs until shooter velocity and hood angle are within tolerance for 5 cycles, then command ends but leaves shooter and hood moving
   * <p> runForever = command runs until interrupted, turn off hood and shooter motor when interrupted
   * @param driveTrain driveTrain subsystem
   * @param shooter shooter subsystem
   * 
   */
  public ShooterScorePrep(DoubleSupplier velocityGetter, DoubleSupplier angleGetter, EndType type, Shooter shooter, Turret turret, DriveTrain driveTrain, Field field) {
   this.driveTrain = driveTrain;
   this.shooter = shooter;
   this.turret = turret;
   this.field = field;
   this.velocityGetter = velocityGetter;
   this.angleGetter = angleGetter;
   this.type = type;
   useGetter = true;
   fromElastic = false;

    addRequirements(shooter);
  }

  /**
   * Sets the hood angle and shooter velocity for scoring.
   * @param velocity shooter motor target velocity, in rpm
   * @param angle hood target angle, in degrees
   * @param type
   * <p> waitForSetPoint = command runs until shooter velocity and hood angle are within tolerance for 5 cycles, then command ends but leaves shooter and hood moving
   * <p> runForever = command runs until interrupted, turn off hood and shooter motor when interrupted
   * <p> endInstantly = command ends instantly
   * @param driveTrain driveTrain subsystem
   * @param shooter shooter subsystem
   * 
   */
  public ShooterScorePrep(double velocity, double angle, EndType type, Shooter shooter, Turret turret, DriveTrain driveTrain, Field field) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.turret = turret;
    this.field = field;
    this.targetVelocity = velocity;
    this.targetHoodAngle = angle;
    this.type = type;
    fromElastic = false;

    addRequirements(shooter);
  }

  /**
   * Sets the hood angle and shooter velocity for scoring, based on values specified on the dashboard. 
   * @param type
   * <p> waitForSetPoint = command runs until shooter velocity and hood angle are within tolerance for 5 cycles, then command ends but leaves shooter and hood moving
   * <p> runForever = command runs until interrupted, turn off hood and shooter motor when interrupted
   * <p> endInstantly = command ends instantly
   * @param shooter shooter subsystem
   * @param driveTrain driveTrain subsystem
   * 
   */
  public ShooterScorePrep(boolean fromElastic, EndType type, Shooter shooter, Turret turret, DriveTrain driveTrain, Field field) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.turret = turret;
    this.field = field;
    this.type = type;
    this.fromElastic = fromElastic;

    if (fromElastic) {
      if (SmartDashboard.getNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
      }

      if (SmartDashboard.getNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, -9999.9) == -9999.9) {
        SmartDashboard.putNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0);
      }
    } else {
      selfCalculate = true;
    }

    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == EndType.waitForSetPoint) shooter.enableFastLogging(true);

    if (fromElastic) {
      targetHoodAngle = SmartDashboard.getNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
      targetVelocity = SmartDashboard.getNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0);
    }
    else if (useGetter) {
      targetHoodAngle = angleGetter.getAsDouble();
      targetVelocity = velocityGetter.getAsDouble();
    }
    // Calculates velocity and angle based on calculated distance to the goal pose
    else if (selfCalculate) { 
      Pose2d robotPose = driveTrain.getPose();
      Translation2d turretTranslation2d = turret.getTurretAbsolutePose(robotPose).getTranslation();

      if (!turret.getShootOnTheMoveEnabled()) {
        goalTranslation2d = field.getStaticShotTarget(turretTranslation2d);
      } else {
        goalTranslation2d = field.getGhostShotTarget(field.getStaticShotTarget(turretTranslation2d), turretTranslation2d, driveTrain.getChassisSpeeds());
      }

      distanceToTarget = goalTranslation2d.getDistance(turretTranslation2d);
      
      // Target shooter motor speed based on: A0 + (A1)(Dist) where A0, A1 are unique constants
      double calculatedVelocity = (ShooterConstants.shooterDistanceToRPMIntercept + (ShooterConstants.shooterDistanceToRPMSlope * distanceToTarget));
      targetVelocity = MathUtil.clamp(calculatedVelocity, 0, ShooterConstants.maxShootRPM);
      
      if (goalTranslation2d == FieldConstants.leftPassBlue || goalTranslation2d == FieldConstants.rightPassRed || goalTranslation2d == FieldConstants.rightPassBlue || goalTranslation2d == FieldConstants.leftPassRed) {
        targetHoodAngle = ShooterConstants.hoodPassAngle;
      } else {
        targetHoodAngle = ShooterConstants.hoodShootAngle;
      } 
    }

    shooter.setHoodAnglePID(targetHoodAngle);
    shooter.setShooterVelocity(targetVelocity);

    // LEDEventUtil.sendEvent(LEDStripEvents.PREPPING_SHOOTER);
    ledSentShooterReadyEvent = false;

    toleranceCounter = 0;
    DataLogUtil.writeMessage("ShooterScorePrep: Initialize, Angle Setpoint =", targetHoodAngle, ", Velocity Setpoint =", targetVelocity,
                             ", Self Calculate =", selfCalculate + ", type =", type + ", initial distance = " + distanceToTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useGetter && type == EndType.runForever) {
      // Don't update angle from getter if type == waitForAngle, otherwise the getter could
      // change the angle a little and never settle on a value. Then the PID controller would
      // try to track and maybe never get in tolerance, and then this command would never end.
      targetHoodAngle = angleGetter.getAsDouble();
      targetVelocity = velocityGetter.getAsDouble();
      shooter.setShooterVelocity(targetVelocity);
    }
    if (fromElastic) {
      shooter.setShooterVelocity(targetVelocity);
    }

    else if (selfCalculate) { 
      Translation2d turretTranslation2d = turret.getTurretAbsolutePose(driveTrain.getPose()).getTranslation();
      if (!turret.getShootOnTheMoveEnabled()) {
        goalTranslation2d = field.getStaticShotTarget(turretTranslation2d);
      } else {
        goalTranslation2d = field.getGhostShotTarget(field.getStaticShotTarget(turretTranslation2d),
                                                      turretTranslation2d,
                                                      driveTrain.getRobotSpeeds());
      }

      distanceToTarget = goalTranslation2d.getDistance(turretTranslation2d);
      
      // Target shooter motor speed based on: A0 + (A1)(Dist) where A0, A1 are unique constants
      double calculatedVelocity = (ShooterConstants.shooterDistanceToRPMIntercept + (ShooterConstants.shooterDistanceToRPMSlope * distanceToTarget));
      targetVelocity = MathUtil.clamp(calculatedVelocity, 0, ShooterConstants.maxShootRPM);
      
      if (goalTranslation2d == FieldConstants.leftPassBlue || goalTranslation2d == FieldConstants.rightPassRed || goalTranslation2d == FieldConstants.rightPassBlue || goalTranslation2d == FieldConstants.leftPassRed) {
        targetHoodAngle = ShooterConstants.hoodPassAngle;
      } else {
        targetHoodAngle = ShooterConstants.hoodShootAngle;
      }

      shooter.setShooterVelocity(targetVelocity);
    }

    if (!ledSentShooterReadyEvent && (shooter.isHoodAtGoalAngle() && shooter.isShooterAtGoalVelocity())) {

      // LEDEventUtil.sendEvent(LEDStripEvents.SHOOTER_READY);
      ledSentShooterReadyEvent = true;
    }

    // Call hoodPID every execute cycle, so that hood continues to run under PID control.
    shooter.setHoodAnglePID(targetHoodAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);

    // Hood is no longer under PID control, so trun off motors
    shooter.setHoodVoltageBothMotors(0);

    if (type == EndType.runForever) {
      shooter.stopShooterMotor();
      // CommandScheduler.getInstance().schedule(new HoodSetAngle(ShooterConstants.hoodMinimumAngle, EndType.waitForSetPoint, shooter));
    }

    DataLogUtil.writeMessage("ShooterScorePrep: End");
    // LEDEventUtil.sendEvent(LEDEventUtil.LEDStripEvents.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (type) {
      case runForever:
        return false;
      case waitForSetPoint:
         if ( shooter.isHoodAtGoalAngle() && shooter.isShooterAtGoalVelocity() ) {
          toleranceCounter++;
        } else {
          toleranceCounter = 0;
        }
        return toleranceCounter >= 5;
      default:
        return true;
    }
  }
}
