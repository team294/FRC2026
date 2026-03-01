// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;

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
}