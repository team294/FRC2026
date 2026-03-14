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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberManualControl extends Command {
  private final Climber climber;
  private final CommandXboxController xboxController;
  
  private boolean rightJoystick;

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogPct = new DoubleLogEntry(log, "/ClimberManualControl/ClimbPercent");

  /**
   * Controls the climber using the Xbox controller joysticks.
   * @param xboxController Xbox controller
   * @param climber Climber subsystem
   * @param rightJoystick true = use right joystick, false = use left joystick
   */
  public ClimberManualControl(CommandXboxController xboxController, Climber climber, boolean rightJoystick) {
    this.climber = climber;
    this.xboxController = xboxController;
    
    this.rightJoystick = rightJoystick;
    addRequirements(climber);

    // Prime data logging at boot time
    long timeNow = RobotController.getFPGATime();
    dLogPct.append(-1, timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("ClimberManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbPercent = rightJoystick ? -xboxController.getRightY() : -xboxController.getLeftY();
    if ((Math.abs(climbPercent) < OIConstants.joystickDeadband)) climbPercent = 0;

    // Slow the control down
    climbPercent *= ClimberConstants.maxManualPercentOutput;
    climber.setClimberPercentOutput(climbPercent);

    // Log data
    long timeNow = RobotController.getFPGATime();
    dLogPct.append(climbPercent, timeNow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotors();

    DataLogUtil.writeMessage("ClimberManualControl: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
