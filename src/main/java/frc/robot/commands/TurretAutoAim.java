// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class TurretAutoAim extends Command {
  private final Turret turret;
  private final DriveTrain driveTrain;
  private final Alliance alliance;

  private Pose2d robotPose2d;
  private Pose2d staticTargetPose2d;
  
  // private Pose2d virtualTargetPose2d;
  // private ChassisSpeeds robotSpeeds;
  // private double distanceToHub;
  // private double estBallTimeOfFlight;
  // private double virtualX;
  // private double virtualY;

  /**
   * Calculate target turret angle for shooting depending on robot position on field
   * @param turret Turret subsystem
   * @param driveTrain DriveTrain subsystem
   * @param alliance current Alliance (Alliance.Blue, Alliance.Red)
  */
  public TurretAutoAim(Turret turret, DriveTrain driveTrain, Alliance alliance) {
    this.turret = turret;
    this.driveTrain = driveTrain;
    this.alliance = alliance;

    addRequirements(turret);  // can't require drivetrain here b/c it would cancel DriveWithJoysticksAdvanced
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    robotPose2d = driveTrain.getPose();
    // robotSpeeds = driveTrain.getRobotSpeeds();
    // distanceToHub = turret.getTurretAbsolutePose(robotPose2d).getTranslation().getDistance(staticTargetPose2d.getTranslation());
    // estFuelTimeOfFlight = distanceToHub / ShooterConstants.fuelExitVelocity;

    if (alliance == Alliance.Blue) {
      if (robotPose2d.getX() < FieldConstants.hubPositionBlue.getX()) {staticTargetPose2d = FieldConstants.hubPositionBlue;}
      else {
        if (robotPose2d.getY() < FieldConstants.rightPassBlue.getY()) {staticTargetPose2d = FieldConstants.rightPassBlue;}
        else {staticTargetPose2d = FieldConstants.leftPassBlue;}
      }
    } else {
      if (robotPose2d.getX() > FieldConstants.hubPositionRed.getX()) {staticTargetPose2d = FieldConstants.hubPositionRed;}
      else {
        if (robotPose2d.getY() < FieldConstants.leftPassRed.getY()) {staticTargetPose2d = FieldConstants.leftPassRed;}
        else {staticTargetPose2d = FieldConstants.rightPassRed;}
      }
    }

    turret.setTurretAngle(turret.getAutoTargetAngleTurretRelative(robotPose2d, staticTargetPose2d));
    // turret.setTurretAngle(turret.getAutoTargetAngleTurretRelative(robotPose2d, virtualTargetPose2d));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !turret.getAutoAimEnabled();
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    turret.stopTurret();
  }
}