// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveCalibration extends Command {
  private DriveTrain driveTrain;
  
  private final double alignTime = 1.0; // Align wheels for 1.0 second before starting ramp
  private double angleFacing, percentOutput, maxPercentOutput, endTime, rampRate;
  private final Timer timer = new Timer();

  /**
   * Drives the robot straight with a ramp velocity, starting at 0 speed.
   * Waits 1 second to align wheel facings prior to starting robot movement, then ramps up speed for "rampTime" seconds.
   * @param angleFacing desired wheel facing relative to front of chassis, in degrees, from -180 to +180 (positive = left, negative = right, 0 = facing front of robot)
   * @param maxPercentOutput percent output, between 0 and 1
   * @param rampTime ramp up time, in seconds
   * @param rampRate ramp rate, in percent output per second 
   * @param driveTrain DriveTrain subsystem
   */
  public DriveCalibration(double angleFacing, double maxPercentOutput, double rampTime, double rampRate, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.angleFacing = angleFacing;
    this.maxPercentOutput = maxPercentOutput;
    this.endTime = rampTime + alignTime;
    this.rampRate = rampRate;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SignalLogger.start();

    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(true);

    DataLogUtil.writeMessage("DriveCalibration: Init, maxPctOut =", maxPercentOutput, ", rampTime =", endTime, ", rampRate =", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    driveTrain.setWheelFacings(angleFacing);

    if (timer.hasElapsed(alignTime)) {
      percentOutput = MathUtil.clamp((currTime-alignTime) * rampRate, -maxPercentOutput, maxPercentOutput);
      driveTrain.setDriveMotorsOutput(percentOutput);
    } else {
      driveTrain.setDriveMotorsOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    driveTrain.enableFastLogging(false);
    timer.stop();
    SignalLogger.stop();

    DataLogUtil.writeMessage("DriveCalibration: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(endTime);
  }
}
