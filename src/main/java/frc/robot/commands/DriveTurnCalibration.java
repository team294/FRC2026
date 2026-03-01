// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveTurnCalibration extends Command {

  private DriveTrain driveTrain;
  
  private double percentOutput, maxPercentOutput, rampTime, rampRate;
  private final Timer timer = new Timer();

  /**
   * Command for gathering data for calibrating the turning motors.
   * @param maxPercentOutput percent output, between 0 and 1
   * @param rampTime ramp up time, in seconds
   * @param rampRate Ramp rate in percent output per second 
   * @param driveTrain DriveTrain subsystem
   */
  public DriveTurnCalibration(double maxPercentOutput, double rampTime, double rampRate, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.maxPercentOutput = maxPercentOutput;
    this.rampTime = rampTime;
    this.rampRate = rampRate;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(true);
    DataLogUtil.writeMessage("DriveTurnCalibration: Init, maxPctOut =", maxPercentOutput, ", rampTime =", rampTime, ", rampRate =", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();
    percentOutput = MathUtil.clamp(currTime*rampRate, -maxPercentOutput, maxPercentOutput);
    driveTrain.setTurningMotorsOutput(percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    driveTrain.enableFastLogging(false);
    DataLogUtil.writeMessage("DriveTurnCalibration: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(rampTime);
  }
}
