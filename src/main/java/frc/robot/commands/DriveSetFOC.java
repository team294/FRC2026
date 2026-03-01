// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.DataLogUtil;

public class DriveSetFOC extends Command {
  private final DriveTrain driveTrain;
  
  private final boolean setFOC;

  /**
   * Sets the drive motor to FOC or trapezoidal commuatation mode.
   * <p><b>NOTE:</b> This takes effect for the <b>next</b> request sent to the motor.
   * @param setFOC true = FOC mode, false = trapezoidal mode
   * @param driveTrain DriveTrain subsystem
   */
  public DriveSetFOC(boolean setFOC, DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    
    this.setFOC = setFOC;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveMotorsFOC(setFOC);
    DataLogUtil.writeMessage("DriveSetFOC: Init, FOC Mode =", setFOC);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  } 
}
