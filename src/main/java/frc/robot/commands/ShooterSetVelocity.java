// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class ShooterSetVelocity extends Command {
  private double velocity = 0.0;
  private final Shooter shooter;
  private final VelocityType type;
  private boolean fromShuffleboard;
  private int counter;

  private DoubleSupplier velocityGetter;
  private boolean useGetter = false;

  public enum VelocityType{
    immediatelyEnd,
    runForever, 
    waitForVelocity
  }
  
  /**
   * Sets the shooter wheel velocity (rpm) for top and bottom shooter motors.
   * @param velocity wheel velocity for top and bottom, in rpm  (+ = shoot forward, - = backwards)
   * @param type 
   * <p> immediatelyEnd = command ends immediately, leave shooter running
   * <p> waitForVelocity = command runs until shooter is within tolerance of the set velocity for 5 cycles, then command ends but leaves shooter running
   * <p> runForever = command runs until interrupted, turn off shooter when interrupted
   * @param shooter shooter subsystem
   */
  public ShooterSetVelocity(double velocity, VelocityType type, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocity = velocity;
    this.type = type;
    this.shooter = shooter;
    this.fromShuffleboard = false;
    addRequirements(shooter);
  }

  /**
   * Sets the shooter wheel velocity (rpm) for top and bottom shooter motors.
   * @param velocityGetter DoubleSupplier that returns the wheel velocity for top and bottom, in rpm  (+ = shoot forward, - = backwards)
   * @param type 
   * <p> immediatelyEnd = command ends immediately, leave shooter running
   * <p> waitForVelocity = command runs until shooter is within tolerance of the set velocity for 5 cycles, then command ends but leaves shooter running
   * <p> runForever = command runs until interrupted, turn off shooter when interrupted
   * @param shooter shooter subsystem
   */
  public ShooterSetVelocity(DoubleSupplier velocityGetter, VelocityType type, Shooter shooter) {
    this.velocityGetter = velocityGetter;
    this.type = type;
    this.shooter = shooter;
    this.fromShuffleboard = false;
    this.useGetter = true;
    addRequirements(shooter);
  }

  /**
   * Sets the shooter wheel velocity (rpm) based upon input from Shuffleboard (+ = shoot forward, - = backwards).
   * @param type ShooterSetVelocity.VelocityType = immediatelyEnd, runForever, or waitForVelocity
   * @param shooter shooter subsystem
   */
  public ShooterSetVelocity(VelocityType type, Shooter shooter) {
    this.velocity = 0.0;
    this.type = type;
    this.shooter = shooter;
    this.fromShuffleboard = true;
    addRequirements(shooter);

    if(SmartDashboard.getNumber(SHOOTER_CMD_SET_VELOCITY_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == VelocityType.waitForVelocity) {
      shooter.enableFastLogging(true);
      //SignalLogger.start();     // Uncomment for Phoenix logging
    }

    if (fromShuffleboard) {
      velocity = SmartDashboard.getNumber(SHOOTER_CMD_SET_VELOCITY_INPUT.value, 0.0);
    }

    if (useGetter) {
      velocity = velocityGetter.getAsDouble();
    }

    shooter.setShooterVelocity(velocity);
    counter = 0;
    DataLogUtil.writeMessage("ShooterSetVelocity: Initialize, Velocity RPM =", velocity, ", type =", type);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useGetter && type == VelocityType.runForever) {
      // Don't update velocity from getter if type == waitForVelocity, otherwise the getter could
      // change the velocity up/down a little (due to noise from vision) and never settle on
      // a value.  Then the PID controller would try to track and maybe never get in tolerance,
      // and then this command would never end.
      velocity = velocityGetter.getAsDouble();
      shooter.setShooterVelocity(velocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);
    SignalLogger.stop();

    if (type == VelocityType.runForever) {
      shooter.stopShooterMotor();
    }
    DataLogUtil.writeMessage("ShooterSetVelocity: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (type){
      case immediatelyEnd:
        return true;
      case runForever:
        return false;
      case waitForVelocity:
        if( (Math.abs(shooter.getShooterVelocity() - velocity) < ShooterConstants.velocityErrorTolerance) ) {
          counter++;
        }else{
          counter = 0;
        }
        return counter >= 5;
      default:
        return true;
    }
  }
}