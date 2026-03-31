// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.subsystems.Feeder;

public class FeederFeedShooter extends Command {
  private final Feeder feeder;
  private final Timer timer;        // track time without Fuel
  private final Timer reverseTimer; // track time spinning in reverse
  private boolean reversed; // track if spindexer reversed so we don't update the setpoint every loop
  private boolean uptakeWithinTolerance; // don't start spindexer until uptake is ready
  
  /**
   * Spins uptake then spindexer to feed Fuel into shooter, reversing spindexer temporarily
   * if no Fuel is detected for a certain amount of time specified in Constants
   * @param feeder Feeder subsystem
   */
  public FeederFeedShooter(Feeder feeder) {
    this.feeder = feeder;
    this.timer = new Timer();
    this.reverseTimer = new Timer();

    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reversed = false;  // guaranteed to start normal direction

    uptakeWithinTolerance = false;
    feeder.setUptakeVelocity(UptakeConstants.uptakeShootRPM);
    timer.restart();  // start fuel tracking timer
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!uptakeWithinTolerance) {
      if (Math.abs(feeder.getUptakeVelocityPIDError()) > FeederConstants.uptakeVelocityToleranceToStartSpindexer) {
        feeder.setSpindexerVelocity(feeder.getUptakeMotorVelocity() * FeederConstants.spindexderToUptakeRPMRatio);
      } else {
        uptakeWithinTolerance = true;
        feeder.setSpindexerVelocity(SpindexerConstants.spindexerShootRPM);
      }
    } else {

      // time without Fuel is 0 if we have Fuel, so reset
      if (feeder.isDetected(0)) {  // check exit sensor b/c entry could have a Fuel while jammed 
        timer.reset();
      } 

      // forwards but no fuel? spin in reverse
      if (!reversed && timer.get() >= FeederConstants.noFuelSeconds) {
        feeder.setSpindexerPercentOutput(SpindexerConstants.spindexerReversePercent);
        reversed = true;
        reverseTimer.restart();  // start tracking time spent in reverse 
      }

      // reversed for long enough, time to start moving forwards again
      else if (reversed && reverseTimer.get() >= FeederConstants.reverseSpindexerSeconds) {
        feeder.setSpindexerVelocity(SpindexerConstants.spindexerShootRPM);
        timer.reset();  // give it time to spin up before we start caring about Fuel again
        reversed = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopSpindexer();
    feeder.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
