// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.UncheckedIOException;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

/** BCR Field class (implementation in progress) */
public class Field {
    private AllianceSelection allianceSelection;
    private AprilTagFieldLayout aprilTagFieldLayout;

    /**
     * Constructor for Field objects.
     * @param allianceSelection
     */
    public Field(AllianceSelection allianceSelection) {
        this.allianceSelection = allianceSelection;

        // Create aprilTagFieldLayout for pose estimators and photonvision
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); 
            DataLogUtil.writeMessage("Field Constructor:  Loaded april tags from file");
        } catch (UncheckedIOException e) {
            RobotPreferences.recordStickyFaults("Field");
            DataLogUtil.writeMessage("Field Constructor:  Error loading april tags from file");
            e.printStackTrace();
        }
    }

    /**
     * Returns the AprilTag field layout for the current season and field
     * @return
     */
    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return aprilTagFieldLayout;
    }

    /**
     * Get target to aim at based on current robot location. <b> Does NOT account for robot moving, use getBallTarget() instead if needed. </b>
     * @param robotTranslation2d Translation2d object of robot position.
     * @return Translation2d object of target.
     */
    public Translation2d getStaticShotTarget(Translation2d robotTranslation2d) {
        if (allianceSelection.getAlliance() == Alliance.Blue) {
            if (robotTranslation2d.getX() < FieldConstants.hubPositionBlue.getX()) {return FieldConstants.hubPositionBlue;}
            else {
                if (robotTranslation2d.getY() < FieldConstants.rightPassBlue.getY()) {return FieldConstants.rightPassBlue;}
                else {return FieldConstants.leftPassBlue;}
            }
        } else {
            if (robotTranslation2d.getX() > FieldConstants.hubPositionRed.getX()) {return FieldConstants.hubPositionRed;}
            else {
                if (robotTranslation2d.getY() < FieldConstants.leftPassRed.getY()) {return FieldConstants.leftPassRed;}
                else {return FieldConstants.rightPassRed;}
            }
        }
    }

    public Translation2d getGhostShotTarget(Translation2d staticTargetTranslation2d, Translation2d turretTranslation2d, ChassisSpeeds robotFieldRelativeSpeeds, double accelerationX, double accelerationY) {
        double offsetX = (robotFieldRelativeSpeeds.vxMetersPerSecond * FieldConstants.timeOfFlightSlope) + FieldConstants.timeOfFlightIntercept;
        double offsetY = (robotFieldRelativeSpeeds.vyMetersPerSecond * FieldConstants.timeOfFlightSlope) + FieldConstants.timeOfFlightIntercept;

        return new Translation2d(staticTargetTranslation2d.getX() + offsetX, staticTargetTranslation2d.getY() + offsetY);
    }
}