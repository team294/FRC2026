// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class TurretDebug extends Command {
  private final Turret turret;
  private int loopCounter;

  /**
   * Pulses turret motor on and off at a low power indefinitely and reads all encoders at each stopped position.
   * 
   * @param turret Turret subsystem
  */
  public TurretDebug(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if (loopCounter <= 4) {
      if (loopCounter == 0) {
        turret.setTurretPercentOutput(0.5);
      }
    } else if (loopCounter == 5) {
      turret.stopTurret();
    } else if (loopCounter == 9) {
      loopCounter = -1;
    }

    loopCounter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
