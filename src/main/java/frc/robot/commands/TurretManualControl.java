// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.DataLogUtil;

public class TurretManualControl extends Command {
  private final Turret turret;
  private final CommandXboxController xboxController;

  private final DataLog log = DataLogManager.getLog();
  private long timeNow = 0;
  
  private final DoubleLogEntry dLogTurretAngle = new DoubleLogEntry(log, "TurretManualControl/Angle");
  private final DoubleLogEntry dLogSetPercent = new DoubleLogEntry(log, "TurretManualControl/SetPercent");

  /**
   * Controls the turret using the Xbox controller joysticks.
   * @param xboxController Xbox controller
   * @param turret Turret subsystem
   */
  public TurretManualControl(CommandXboxController xboxController, Turret turret) {
    this.turret = turret;
    this.xboxController = xboxController;
    
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("TurretManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turretPercent = -xboxController.getRightX();
    if ((Math.abs(turretPercent) < OIConstants.joystickDeadband)) turretPercent = 0;

    // Slow the control down
    turretPercent *= TurretConstants.maxManualPercentOutput;
    turret.setTurretPercentOutput(turretPercent);

    timeNow = RobotController.getFPGATime();
    dLogTurretAngle.append(turret.getTurretAngle(), timeNow);
    dLogSetPercent.append(turretPercent, timeNow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();

    DataLogUtil.writeMessage("TurretManualControl: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
