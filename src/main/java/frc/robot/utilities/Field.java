// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

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

    /**
     * Get "ghost" target to use for aiming shots to account for robot acceleration and velocity
     * @param staticTargetTranslation2d Translation2d object of shot target
     * @param turretTranslation2d Translation2d object of turret (shooter)
     * @param robotFieldRelativeSpeeds <i>Field-relative</i> robot velocities
     * @param accelerationX <i>Field-relative</i> robot acceleration in x-direction (away from Blue POV)
     * @param accelerationY <i>Field-relative</i> robot acceleration in y-direction (left from Blue POV)
     * @return Translation2d object of "ghost" target to aim at
     */
    public Translation2d getGhostShotTarget(Translation2d staticTargetTranslation2d, Translation2d turretTranslation2d, ChassisSpeeds robotFieldRelativeSpeeds, double accelerationX, double accelerationY) {
        double estTimeOfFlight = turretTranslation2d.getDistance(staticTargetTranslation2d) * FieldConstants.timeOfFlightSlope + FieldConstants.timeOfFlightIntercept;

        // final position = 0.5(acceleration)(time^2) + (velocity)(time) + (initial position)
        // acceleration and velocity are done here
        double offsetX = (0.5 * accelerationX * estTimeOfFlight * estTimeOfFlight) + (robotFieldRelativeSpeeds.vxMetersPerSecond * estTimeOfFlight);
        double offsetY = (0.5 * accelerationY * estTimeOfFlight * estTimeOfFlight) + (robotFieldRelativeSpeeds.vyMetersPerSecond * estTimeOfFlight);

        // add to initial position to get final position
        return new Translation2d(staticTargetTranslation2d.getX() + offsetX, staticTargetTranslation2d.getY() + offsetY);
    }
}