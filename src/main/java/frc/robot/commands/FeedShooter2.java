// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.subsystems.Feeder;

public class FeedShooter2 extends Command {
  private final Feeder feeder;
  private int counter, counterReverse;
  private boolean uptakeWithinTolerance;
  
  /**
   * Agitates the intake up and down repeatedly, ending only if interrupted
   * and moving the intake down to intaking position when interrupted
   * @param intake Intake subsystem
   */
  public FeedShooter2(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    counterReverse = 0;
    uptakeWithinTolerance = false;
    feeder.setUptakeVelocity(UptakeConstants.uptakeShootRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!uptakeWithinTolerance && Math.abs(feeder.getUptakeVelocityPIDError()) < 50.0) {
      uptakeWithinTolerance = true;
      feeder.setSpindexerVelocity(SpindexerConstants.spindexerShootRPM);
    } else if (uptakeWithinTolerance) {
      if (feeder.isDetected(0)) {
        counter = 0;
      } else if (counterReverse >= 25) {
        feeder.setSpindexerVelocity(SpindexerConstants.spindexerShootRPM);
        counterReverse = 0;
        counter = 0;
      } else if (counter >= 50) {
        feeder.setSpindexerPercentOutput(SpindexerConstants.spindexerReversePercent);
        counterReverse++;
        counter++;
      } else {
        counter++;
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
