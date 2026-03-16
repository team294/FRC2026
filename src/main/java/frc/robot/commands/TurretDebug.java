// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretDebug extends Command {
  private final Turret turret;
  private int loopCounter;
  private final DataLog log = DataLogManager.getLog();
  private long timeNow = 0;
  
  private final DoubleLogEntry dLogRemoteEncoder1Raw = new DoubleLogEntry(log, "TurretDebug/Remote Encoder 1 Raw");
  private final DoubleLogEntry dLogRemoteEncoder2Raw = new DoubleLogEntry(log, "TurretDebug/Remote Encoder 2 Raw");
  private final DoubleLogEntry dLogMotorEncoderRaw = new DoubleLogEntry(log, "TurretDebug/Motor Encoder Raw");
  private final DoubleLogEntry dLogRemoteEncoder1Data = new DoubleLogEntry(log, "TurretDebugData/Remote Encoder 1 Raw");
  private final DoubleLogEntry dLogRemoteEncoder1Abs = new DoubleLogEntry(log, "TurretDebugData/Remote Encoder 1 Abs");
  private final DoubleLogEntry dLogRemoteEncoder2Data = new DoubleLogEntry(log, "TurretDebugData/Remote Encoder 2 Raw");
  private final DoubleLogEntry dLogRemoteEncoder2Abs = new DoubleLogEntry(log, "TurretDebugData/Remote Encoder 2 Abs");
  private final DoubleLogEntry dLogMotorEncoderData = new DoubleLogEntry(log, "TurretDebugData/Motor Encoder Raw");

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
    timeNow = RobotController.getFPGATime();
    dLogRemoteEncoder1Raw.append(turret.getCANcoderRotationsRaw(1), timeNow);
    dLogRemoteEncoder2Raw.append(turret.getCANcoderRotationsRaw(2), timeNow);
    dLogMotorEncoderRaw.append(turret.getTurretEncoderRotationsRaw(), timeNow);

      if (loopCounter <= 4) {
      if (loopCounter == 0) {
        turret.setTurretPercentOutput(0.5);
      }
    } else if (loopCounter == 5) {
      turret.stopTurret();
    } else if (loopCounter == 9) {
      dLogRemoteEncoder1Data.append(turret.getCANcoderRotationsRaw(1), timeNow);
      dLogRemoteEncoder1Abs.append(turret.getCANcoderAbsRotationsRaw(1), timeNow);
      dLogRemoteEncoder2Data.append(turret.getCANcoderRotationsRaw(2), timeNow);
      dLogRemoteEncoder2Abs.append(turret.getCANcoderAbsRotationsRaw(2), timeNow);
      dLogMotorEncoderData.append(turret.getTurretEncoderRotationsRaw(), timeNow);
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
