// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberCalibration extends Command {
  private Climber climber;
  
  private double percentOutput, rampRate;
  private final Timer timer = new Timer();

  private enum CalibrationRoutineState {
    RAMP_UP,
    STOP_UP,
    RAMP_DOWN,
    STOP_DOWN,
  }
  private CalibrationRoutineState state;

  /**
   * Ramps climber speed upwards and then downwards, reversing when 10 inches from top or bottom.
   * The data collected during this routine can be used to calculate the terms of the climber profile.
   * @param rampRate Ramp rate in percent output/second 
   * @param climber Climber subsystem
   */
  public ClimberCalibration(double rampRate, Climber climber) {
    this.climber = climber;
    
    this.rampRate = rampRate;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    state = CalibrationRoutineState.RAMP_UP;

    climber.enableFastLogging(true);
    DataLogUtil.writeMessage("ClimberCalibration: Init, rampRate =", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    if (!climber.isClimberCalibrated()) return;

    switch (state) {
      // Ramp upwards until 10 inches from top
      case RAMP_UP:
        if (climber.getClimberPosition() < ClimberPosition.UPPER_LIMIT.value - 10.0) {
          percentOutput = MathUtil.clamp(currTime * rampRate, -1.0, 1.0);
          climber.setClimberPercentOutput(percentOutput);      
        } else {
          state = CalibrationRoutineState.STOP_UP;
          timer.reset();
          timer.start();
        }
        break;

      // Stop motor for 2 seconds
      case STOP_UP:
        if (currTime < 2.0) {
          climber.stopClimberMotors();     
        } else {
          state = CalibrationRoutineState.RAMP_DOWN;
          timer.reset();
          timer.start();
        }
        break;
      
      // Ramp downwards until 10 inches from bottom
      case RAMP_DOWN:
        if (climber.getClimberPosition() > ClimberConstants.ClimberPosition.LOWER_LIMIT.value + 10.0) {
          percentOutput = MathUtil.clamp(-currTime * rampRate, -1.0, 1.0);
          climber.setClimberPercentOutput(percentOutput);      
        } else {
          state = CalibrationRoutineState.STOP_DOWN;
          timer.reset();
          timer.start();
        }
        break;

      // Stop motor for 2 seconds
      case STOP_DOWN:
      default:
        if (currTime < 2.0) {
          climber.stopClimberMotors();     
        } else {
          state = CalibrationRoutineState.RAMP_UP;
          timer.reset();
          timer.start();
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotors();
    climber.enableFastLogging(false);
    DataLogUtil.writeMessage("ClimberCalibration: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
