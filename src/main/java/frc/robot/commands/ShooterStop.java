// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataLogUtil;

public class ShooterStop extends Command {
  private final Shooter shooter;
  
 
  /**
   * Sets the percent output of the shooter to 0 and ends immediately.
   * @param shooter Shooter subsystem
   */
  public ShooterStop(Shooter shooter) {
    this.shooter = shooter;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterPercentOutput(0);
    DataLogUtil.writeMessage("Shooter Stop: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("Shooter Stop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
