// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElasticKeyConstants.ElasticKey;
import frc.robot.Constants.EndType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Field;

public class ShooterScorePrep extends Command {
  private final Shooter shooter;
  private final DriveTrain driveTrain;
  private final Field field;
  private DoubleSupplier velocityGetter;
  private DoubleSupplier angleGetter;
  private boolean shootOnTheMove;
  // private Supplier<Pose2d> poseSupplier;
  private Translation2d goalTranslation2d;
  private double targetVelocity;
  private double targetAngle;
  private boolean useGetter = false;
  private boolean selfCalculate = false;
  private boolean fromElastic;

  private int toleranceCounter = 0;
  private double distanceToTarget;


  private final EndType type;

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
  public ShooterScorePrep(DoubleSupplier velocityGetter, DoubleSupplier angleGetter, EndType type, Shooter shooter, DriveTrain driveTrain,  Field field) {
   this.driveTrain = driveTrain;
   this.shooter = shooter;
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
  public ShooterScorePrep(double velocity, double angle, EndType type, Shooter shooter, DriveTrain driveTrain, Field field) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.field = field;
    this.targetVelocity = velocity;
    this.targetAngle = angle;
    this.type = type;
    fromElastic = false;

    addRequirements(shooter);
  }

  /**
   * Sets the hood angle and shooter velocity for scoring. 
   * @param fromElastic boolean, true = reads values from elastic, false = calculates angle and velocity based on hub distance
   * @param type
   * <p> waitForSetPoint = command runs until shooter velocity and hood angle are within tolerance for 5 cycles, then command ends but leaves shooter and hood moving
   * <p> runForever = command runs until interrupted, turn off hood and shooter motor when interrupted
   * <p> endInstantly = command ends instantly
   * @param driveTrain driveTrain subsystem
   * @param shooter shooter subsystem
   * 
   */
  public ShooterScorePrep(EndType type, Shooter shooter, DriveTrain driveTrain, Field field) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.field = field;
    this.type = type;
    fromElastic = true;
    
    if (SmartDashboard.getNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
    }

    if (SmartDashboard.getNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0);
    }
    addRequirements(shooter);
  }

  /**
   * Calculates and sets the hood angle and shooter velocity for scoring, based on robot's distance from the nearest hub
   * @param allianceSelection the robot's alliance (determines which hub is targeted)
   * @param fromElastic boolean, true = reads values from elastic, false = calculates angle and velocity based on hub distance
   * @param type
   * <p> waitForSetPoint = command runs until shooter velocity and hood angle are within tolerance for 5 cycles, then command ends but leaves shooter and hood moving
   * <p> runForever = command runs until interrupted, turn off hood and shooter motor when interrupted
   * <p> endInstantly = command ends instantly
   * @param driveTrain driveTrain subsystem
   * @param shooter shooter subsystem
   * 
   */
  public ShooterScorePrep(boolean shootOnTheMove, EndType type, Shooter shooter, DriveTrain driveTrain,  Field field) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.field = field;
    this.type = type;
    this.shootOnTheMove = shootOnTheMove;
    selfCalculate = true;

    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == EndType.waitForSetPoint) shooter.enableFastLogging(true);

    if (fromElastic) {
      targetAngle = SmartDashboard.getNumber(ElasticKey.HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
      targetVelocity = SmartDashboard.getNumber(ElasticKey.SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0);
    }
    else if (useGetter) {
      targetAngle = angleGetter.getAsDouble();
      targetVelocity = velocityGetter.getAsDouble();
    }
    // Calculates velocity and angle based on calculated distance to the goal pose
    else if (selfCalculate) { 
      Translation2d robotTranslation2d = driveTrain.getPose().getTranslation();
      if (!shootOnTheMove) {
        goalTranslation2d = field.getStaticShotTarget(robotTranslation2d);
      } else {
        // goalpose = shootonthemove
      }

      distanceToTarget = goalTranslation2d.getDistance(driveTrain.getPose().getTranslation()); // TODO verify units are correct
      
      // Target shooter motor speed based on: A0 + (A1)(Dist) + A2(Dist^2) where A0, A1, and A2 are unique constants
      double calculatedVelocity = (ShooterConstants.shooterDistanceToRPMSlope + (ShooterConstants.shooterDistanceToRPMIntercept * distanceToTarget));
      targetVelocity = MathUtil.clamp(calculatedVelocity, 0, ShooterConstants.maxShootRPM);
      targetAngle = ShooterConstants.hoodShootAngle;
      
      // Target hood angle based on: (shooter speed)(B0) + B1, where B0 and B1 are unique constants
      // targetAngle = ((targetVelocity * ShooterConstants.hoodScoreB0) + ShooterConstants.hoodScoreB1); Not varying hood angle at the moment 
    }

    shooter.setHoodAnglePID(targetAngle);
    shooter.setShooterVelocity(targetVelocity);

    DataLogUtil.writeMessage("ShooterScorePrep: Initialize, Angle Setpoint =", targetAngle, ", Velocity Setpoint =", targetVelocity,
                             ", Self Calculate =", selfCalculate + ", type =", type + ", initial distance = " + distanceToTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (useGetter && type == EndType.runForever) {
      // Don't update angle from getter if type == waitForAngle, otherwise the getter could
      // change the angle a little and never settle on a value. Then the PID controller would
      // try to track and maybe never get in tolerance, and then this command would never end.
      targetAngle = angleGetter.getAsDouble();
      targetVelocity = velocityGetter.getAsDouble();
      shooter.setHoodAnglePID(targetAngle);
      shooter.setShooterVelocity(targetVelocity);
    }
    if (fromElastic) {
      shooter.setHoodAnglePID(targetAngle);
      shooter.setShooterVelocity(targetVelocity);
    }

    else if (selfCalculate){ // Calculates distance and velocity based on position
      Translation2d robotTranslation2d = driveTrain.getPose().getTranslation();
      if (!shootOnTheMove) {
        goalTranslation2d = field.getStaticShotTarget(robotTranslation2d);
      } else {
        // goalpose = shootonthemove
      }

      distanceToTarget = goalTranslation2d.getDistance(driveTrain.getPose().getTranslation()); // TODO verify units are correct
      
      // Target shooter motor speed based on: A0 + (A1)(Dist) + A2(Dist^2) where A0, A1, and A2 are unique constants
      double calculatedVelocity = (ShooterConstants.shooterDistanceToRPMSlope + (ShooterConstants.shooterDistanceToRPMIntercept * distanceToTarget));
      targetVelocity = MathUtil.clamp(calculatedVelocity, 0, ShooterConstants.maxShootRPM);
      targetAngle = ShooterConstants.hoodShootAngle;
      shooter.setHoodAnglePID(targetAngle);
      shooter.setShooterVelocity(targetVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);

    if (type == EndType.runForever) {
      shooter.setHoodVoltageBothMotors(0);
      shooter.stopShooterMotor();
      CommandScheduler.getInstance().schedule(new HoodSetAngle(ShooterConstants.hoodMinimumAngle, EndType.waitForSetPoint, shooter));
    }

    DataLogUtil.writeMessage("ShooterScorePrep: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (type) {
      case runForever:
        return false;
      case waitForSetPoint:
         if ( (Math.abs(shooter.getHoodAngle() - targetAngle) < ShooterConstants.hoodErrorTolerance) && (Math.abs(shooter.getShooterVelocity(1) - targetVelocity) < ShooterConstants.shooterVelocityErrorTolerance)) {
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
