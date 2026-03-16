// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodSetAngle extends Command {
  private double targetAngle = ShooterConstants.hoodMinimumAngle;
  private final Shooter shooter;
  private final boolean fromElastic;
  private int counter;
  
  private DoubleSupplier angleGetter;
  private boolean useGetter = false;

  private final EndType type;

  public enum EndType {
    immediatelyEnd,
    runForever,
    waitForAngle
  }
  
  /**
   * Sets the hood angle (degrees).
   * @param angleGetter DoubleSupplier that returns the target angle for the hood, in degrees
   * @param type
   * <p> waitForAngle = command runs until hood is within tolerance of the set angle for 5 cycles, then command ends but leaves hood moving
   * <p> runForever = command runs until interrupted, turn off hood when interrupted
   * @param shooter shooter subsystem
   */
  public HoodSetAngle(DoubleSupplier angleGetter, EndType type, Shooter shooter) {
    this.angleGetter = angleGetter;
    this.type = type;
    this.shooter = shooter;
    this.fromElastic = false;
    this.useGetter = true;

    addRequirements(shooter);
  }

  /**
   * Sets the hood angle (degrees).
   * @param angle angle for hood, in degrees
   * @param type
   * <p> waitForAngle = command runs until hood is within tolerance of the set angle for 5 cycles, then command ends but leaves hood moving
   * <p> runForever = command runs until interrupted, turn off hood when interrupted
   * @param shooter shooter subsystem
   */
  public HoodSetAngle(double angle, EndType type, Shooter shooter) {
    this.type = type;
    this.shooter = shooter;
    this.fromElastic = false;

    addRequirements(shooter);
  }

  /** Sets the hood angle (degrees).
   * @param type
   * <p> waitForAngle = command runs until hood is within tolerance of the set angle for 5 cycles, then command ends but leaves hood moving
   * <p> runForever = command runs until interrupted, turn off hood when interrupted
   * @param shooter shooter subsystem
   * */
  public HoodSetAngle(EndType type, Shooter shooter) {
    this.type = type;
    this.shooter = shooter;
    this.fromElastic = true;

    if (SmartDashboard.getNumber(HOOD_CMD_SET_ANGLE_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
    }

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == EndType.waitForAngle) {
      shooter.enableFastLogging(true);
    }

    if (fromElastic) targetAngle = SmartDashboard.getNumber(HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);

    if (useGetter) targetAngle = angleGetter.getAsDouble();

    shooter.setHoodAnglePID(targetAngle);

    counter = 0;
    DataLogUtil.writeMessage("HoodSetAngle: Initialize, Angle Setpoint =", targetAngle, ", type =", type);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (useGetter && type == EndType.runForever) {
      // Don't update angle from getter if type == waitForAngle, otherwise the getter could
      // change the angle a little and never settle on a value. Then the PID controller would
      // try to track and maybe never get in tolerance, and then this command would never end.
      targetAngle = angleGetter.getAsDouble();
      shooter.setHoodAnglePID(targetAngle);
    }

    if (fromElastic) {
      if (type == EndType.runForever) {
        targetAngle = SmartDashboard.getNumber(HOOD_CMD_SET_ANGLE_INPUT.value, ShooterConstants.hoodMinimumAngle);
      }
      shooter.setHoodAnglePID(targetAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);

    if (type == EndType.runForever || type == EndType.waitForAngle) {
      shooter.setHoodVoltageBothMotors(0);
    }

    DataLogUtil.writeMessage("HoodSetAngle: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (type) {
      case runForever:
        return false;
      case waitForAngle:
        if ( Math.abs(shooter.getHoodAngle() - targetAngle) < ShooterConstants.hoodErrorTolerance) {
          counter++;
        } else {
          counter = 0;
        }
        return counter >= 5;
      default:
        return true;
    }
  }
}
