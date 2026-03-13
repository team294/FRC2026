// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.SpindexerConstants;
import frc.robot.Constants.FeederConstants.UptakeConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class FeedShooter extends Command {
  private final Intake intake;
  private final Feeder feeder;
  private final Timer timer;
  private boolean currentlyReversed;
  
  /**
   * Spins the uptake and spindexer motor (sequentially in that order), then agitates the intake repeatedly to feed shooter until interrupted.
   * If the command goes a set amount of time without detecting a ball, briefly spins the spindeer in reverse then resets it to normal.
   * When interrupted, stops the uptake and spindexer motors (sequentially in that order) and the Intake is moved to its intaking position
   * @param intake Intake subsystem
   * @param feeder Feeder subsystem
  */
  public FeedShooter(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;

    timer = new Timer();
    currentlyReversed = false;

    addRequirements(intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(
      sequence(
        new UptakeSetVelocity(UptakeConstants.uptakeShootRPM, feeder),
        new SpindexerSetVelocity(SpindexerConstants.spindexerShootRPM, feeder),
        new IntakeDeployAgitate(intake)
      )
    );

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!currentlyReversed && (feeder.isDetected(0) || feeder.isDetected(1))) timer.reset();

    if (timer.get() >= FeederConstants.noFuelSeconds) {
      currentlyReversed = true;
      CommandScheduler.getInstance().schedule(
        new SpindexerSetVelocity(SpindexerConstants.spindexerReverseRPM, feeder)
      );

      timer.reset();
    }

    if (currentlyReversed && timer.get() >= FeederConstants.reverseSpindexerSeconds) {
      CommandScheduler.getInstance().schedule(
        new SpindexerSetVelocity(SpindexerConstants.spindexerShootRPM, feeder)
      );
      
      currentlyReversed = false;
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(
      sequence(
        new UptakeSetPercent(0.0, true, feeder),
        new SpindexerSetPercent(0.0, true, feeder)
      )
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
