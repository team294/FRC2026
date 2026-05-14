// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndType;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.utilities.DataLogUtil;

import static frc.robot.Constants.ElasticKeyConstants.ElasticKey.*;

public class TurretSetAngle extends Command {
  private final Turret turret;
  private final EndType endType;
  private final boolean fromDashboard;

  private DoubleSupplier angleGetter;
  private final boolean useGetter;
  private double targetAngle;

  private int toleranceCounter;

   /**
    * Sets the angle of the turret if the turret is calibrated.  Otherwise, does nothing.
    * 
    * @param angle target angle, in degrees (0 = facing straight back out of robot,
    *              positive = counter clockwise, negative = clockwise)
    * @param endType
    * <p> immediatelyEnd = command ends immediately, but turret continues turning towards requested angle
    * <p> waitForSetPoint = command runs until turret is within tolerance of the requested angle for 5 cycles, then command ends
    * <p> runForever = command runs until interrupted, continuing to move to last requested angle when interrupted.  This
    *     endType is useful when using the angleGetter implementation for tracking a target.
    * @param turret Turret subsystem
    */
  public TurretSetAngle(double angle, EndType endType, Turret turret) {
    this.turret = turret;
    this.endType = endType;
    fromDashboard = false;
    useGetter = false;

    this.targetAngle = angle;

    if (!turret.isTurretCalibrated())  endType = EndType.immediatelyEnd;

    addRequirements(turret);
  }

   /**
    * Sets the angle of the turret if the turret is calibrated.  Otherwise, does nothing.
    * 
    * @param angleGetter DoubleSupplier that returns the target angle, in degrees (0 = facing straight back out of robot,
    *              positive = counter clockwise, negative = clockwise)
    * @param endType
    * <p> immediatelyEnd = command ends immediately, but turret continues turning towards requested angle
    * <p> waitForSetPoint = command runs until turret is within tolerance of the requested angle for 5 cycles, then command ends
    * <p> runForever = command runs until interrupted, continuing to move to last requested angle when interrupted.  This
    *     endType is useful when using the angleGetter implementation for tracking a target.
    * @param turret Turret subsystem
    */
  public TurretSetAngle(DoubleSupplier angleGetter, EndType endType, Turret turret) {
    this.turret = turret;
    this.endType = endType;
    fromDashboard = false;
    useGetter = true;

    this.angleGetter = angleGetter;

    if (!turret.isTurretCalibrated())  endType = EndType.immediatelyEnd;

    addRequirements(turret);
  }

  /** 
    * Sets the angle of the turret if the turret is calibrated.  Otherwise, does nothing.
    * Target angle is provided on the dashboard, in degrees (0 = facing straight back out of robot,
    *              positive = counter clockwise, negative = clockwise)
    * @param endType
    * <p> immediatelyEnd = command ends immediately, but turret continues turning towards requested angle
    * <p> waitForSetPoint = command runs until turret is within tolerance of the requested angle for 5 cycles, then command ends
    * <p> runForever = command runs until interrupted, continuing to move to last requested angle when interrupted.  This
    *     endType is useful when using the angleGetter implementation for tracking a target.
    * @param turret Turret subsystem
   */
  public TurretSetAngle(EndType endType, Turret turret) {
    this.turret = turret;
    this.endType = endType;
    fromDashboard = true;
    useGetter = false;

    if (SmartDashboard.getNumber(TURRET_CMD_SET_ANGLE_INPUT.value, -9999.9) == -9999.9) {
      SmartDashboard.putNumber(TURRET_CMD_SET_ANGLE_INPUT.value, 0.0);
    }
    
    if (!turret.isTurretCalibrated())  endType = EndType.immediatelyEnd;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromDashboard) {
      targetAngle = SmartDashboard.getNumber(TURRET_CMD_SET_ANGLE_INPUT.value, 0.0);
    }

    if (useGetter) {
      targetAngle = angleGetter.getAsDouble();
    }

    //SignalLogger.start();     // Uncomment for Phoenix logging
    turret.setTurretAngle(targetAngle);
    toleranceCounter = 0;

    DataLogUtil.writeMessage("TurretSetAngle: Initialize, Target angle =", targetAngle, ", endType =", endType, 
        ", starting angle =", turret.getTurretAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((useGetter || fromDashboard) && endType == EndType.runForever) {
      // Don't update angle from getter/dashboard if type == waitForVelocity, otherwise the getter could
      // change the velocity up/down a little (due to noise from vision) and never settle on
      // a value.  Then the PID controller would try to track and maybe never get in tolerance,
      // and then this command would never end.
      if (fromDashboard) {
        targetAngle = SmartDashboard.getNumber(TURRET_CMD_SET_ANGLE_INPUT.value, 0.0);
      }

      if (useGetter) {
        targetAngle = angleGetter.getAsDouble();
      }

      turret.setTurretAngle(targetAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SignalLogger.stop();
    DataLogUtil.writeMessage("TurretSetAngle: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (endType) {
      case immediatelyEnd:
        return true;
      case runForever:
        return false;
      case waitForSetPoint:
        if ( (Math.abs(turret.getTurretAngle() - targetAngle) < TurretConstants.angleTolerance) ) {
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
