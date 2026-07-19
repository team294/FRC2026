// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

public class AllianceSelection {
  
  private int logRotationKey;
  private Alliance alliance;  // Red or Blue
  private List<Consumer<Alliance>> newAllianceCalls;

  private enum AllianceChoice {
    Red,
    Blue,
    Auto
  }

  private SendableChooser<AllianceChoice> allianceChooser = new SendableChooser<>();

  public AllianceSelection() {
    
    newAllianceCalls = new ArrayList<Consumer<Alliance>>();
    logRotationKey = DataLogUtil.allocateLogRotation();

		allianceChooser.setDefaultOption("Automatic", AllianceChoice.Auto);
		allianceChooser.addOption("Red", AllianceChoice.Red);
		allianceChooser.addOption("Blue", AllianceChoice.Blue);
	
		// Show alliance selection widget on Elastic
		SmartDashboard.putData("Alliance Selection", allianceChooser);

    // Set Alliance on Elastic
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      setAlliance(ally.get());
      DataLogUtil.writeMessageEcho("Alliance Selection: Init, Alliance from DriverStation =", alliance.name());
    } else {
      setAlliance(Alliance.Blue);
      DataLogUtil.writeMessageEcho("Alliance Selection: Init, DriverStation not present - default to Blue. ", alliance.name());
    }
  }

  /**
   * Sets the current alliance.
   * @param alliance Red, Blue, or Invalid
   */
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    DataLogUtil.writeMessageEcho("Alliance Selection: SetAlliance, Alliance Chooser =", allianceChooser.getSelected().name(), ", Alliance =", alliance.name());

    SmartDashboard.putBoolean("Alliance Blue", alliance != Alliance.Red);
    SmartDashboard.putBoolean("Alliance Red", alliance != Alliance.Blue);
    SmartDashboard.putString("Alliance", alliance.name());
  }

  /**
   * Adds a consumer to the newAllianceCalls list that is called whenever the alliance of the robot is changed.
   * @param newConsumer new Alliance consumer that will be added to the list
   */
  public void addAllianceChangeNotification(Consumer<Alliance> newConsumer) {
    newAllianceCalls.add(newConsumer);
  }

  /**
   * Gets the current alliance.
   * @return Red or Blue
   */
  public Alliance getAlliance() {
    return alliance;
  }

  /**
   * Runs once per scheduler cycle.
   */
  public void periodic() {
    if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
      Alliance newAlliance = alliance;

      switch (allianceChooser.getSelected()) {
        case Auto:
          // Do not auto change alliance in Auto or Teleop mode
          if (DriverStation.isDisabled()) {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) newAlliance = ally.get();
          }
          break;
        case Red:
          newAlliance = Alliance.Red;
          break;
        case Blue:
          newAlliance = Alliance.Blue;
          break;
      }

      if (alliance != newAlliance) {
        setAlliance(newAlliance);
        for (Consumer<Alliance> setters : newAllianceCalls) {
          setters.accept(newAlliance);
        }
      }
    }
  }
}