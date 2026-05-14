// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Field;

public class TurretAutoAim extends Command {
  private final Turret turret;
  private final DriveTrain driveTrain;
  private final Field field;

  private Pose2d robotPose2d;
  private double turretTargetAngle;

  /**
   * Calculate target turret angle for shooting depending on robot position on field
   * @param turret Turret subsystem
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
  */
  public TurretAutoAim(Turret turret, DriveTrain driveTrain, Field field) {
    this.turret = turret;
    this.driveTrain = driveTrain;
    this.field = field;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("TurretAutoAim:  Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    robotPose2d = driveTrain.getPose();
    Translation2d turretTranslation2d = turret.getTurretAbsolutePose(robotPose2d).getTranslation();

    if (turret.getShootOnTheMoveEnabled()) {
      turretTargetAngle = turret.getAutoTargetAngleTurretRelative(
                            robotPose2d, 
                            field.getGhostShotTarget(
                              field.getStaticShotTarget(turretTranslation2d),
                              turret.getTurretAbsolutePose(robotPose2d).getTranslation(),
                              driveTrain.getRobotSpeeds()
                            )
                          );
    } else {
      turretTargetAngle = turret.getAutoTargetAngleTurretRelative(
                            robotPose2d, 
                            field.getStaticShotTarget(turretTranslation2d)
                          );
    }

    if (turret.getAutoAimEnabled()) {
      turret.setTurretAngle(turretTargetAngle);
    }

    SmartDashboard.putNumber("Turret Distance To Blue Hub", turret.getTurretAbsolutePose(robotPose2d).getTranslation().getDistance(FieldConstants.hubPositionBlue));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !turret.getAutoAimEnabled();
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    turret.stopTurret();
    DataLogUtil.writeMessage("TurretAutoAim:  End, interrupted =", interrupted);
  }
}