/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

/**
 * Utility to handle all stored robot preferences.
 */
public class RobotPreferences {
  private static String problemSubsystem = "";
  private static boolean problemExists = false;

  /**
   * Reads preferences stored in the RoboRIO flash memory and updates their values in Constants.
   * If a preference is not found on the RoboRIO, then this method creates that key on the RoboRio
   * using the default values from Constants.
   * Note: Any variables in Constants that are read from Preferences must not be "final".
   */
	public static void readPreferencesToConstants(){
    // Add a row for each preference to read.
    // RobotConstants.prototypeBot = readBoolean("prototypeBot", RobotConstants.prototypeBot);

    DriveConstants.offsetAngleFrontLeftMotor = readDouble("Drive.offsetAngleFrontLeftMotor", DriveConstants.offsetAngleFrontLeftMotor);        
    DriveConstants.offsetAngleFrontRightMotor = readDouble("Drive.offsetAngleFrontRightMotor", DriveConstants.offsetAngleFrontRightMotor);        
    DriveConstants.offsetAngleBackLeftMotor = readDouble("Drive.offsetAngleBackLeftMotor", DriveConstants.offsetAngleBackLeftMotor);        
    DriveConstants.offsetAngleBackRightMotor = readDouble("Drive.offsetAngleBackRightMotor", DriveConstants.offsetAngleBackRightMotor);        
    // DriveConstants.updateDerivedConstants();

    problemSubsystem = readString("problemSubsystem", problemSubsystem);
    problemExists = readBoolean("problemExists", problemExists);
  }

  /**
   * Reads a boolean key from the RoboRIO preferences. If the key does not exist on the 
   * RoboRIO, then this method creates the key on the RoboRIO with the default value
   * (and in that case also returns the default value).
   * @param keyName Name of the RoboRIO preferences key
   * @param defaultValue Default value if the key does not already exist
   * @return Value from RoboRIO preferences, or default value if it did not exist
   */
  private static boolean readBoolean(String keyName, boolean defaultValue) {
    if (!Preferences.containsKey(keyName)) {
      Preferences.setBoolean(keyName, defaultValue);
    } 
    return Preferences.getBoolean(keyName, defaultValue);   
  }

  /**
   * Reads a double key from the RoboRIO preferences. If the key does not exist on the 
   * RoboRIO, then this method creates the key on the RoboRIO with the default value
   * (and in that case also returns the default value).
   * @param keyName Name of the RoboRIO preferences key
   * @param defaultValue Default value if the key does not already exist
   * @return Value from RoboRIO preferences, or default value if it did not exist
   */
  private static double readDouble(String keyName, double defaultValue) {
    if (!Preferences.containsKey(keyName)){
      Preferences.setDouble(keyName, defaultValue);
    } 
    return Preferences.getDouble(keyName, defaultValue);   
  }

  /**
   * Reads a string key from the RoboRIO preferences. If the key does not exist on the 
   * RoboRIO, then this method creates the key on the RoboRIO with the default value
   * (and in that case also returns the default value).
   * @param keyName Name of the RoboRIO preferences key
   * @param defaultValue Default value if the key does not already exist
   * @return Value from RoboRIO preferences, or default value if it did not exist
   */
  private static String readString(String keyName, String defaultValue) {
    if (!Preferences.containsKey(keyName)){
      Preferences.setString(keyName, defaultValue);
    } 
    return Preferences.getString(keyName, defaultValue);   
  }

  /**
   * Records in robotPreferences, log, and Elastic that a problem was found in a subsystem
   * (only records if the subsystem was not already flagged).
   * @param subsystem String name of subsystem in which a problem exists
   */
  public static void recordStickyFaults(String subsystem) {
    // Record that we have a sticky fault
    problemExists = true;

    // Add this subsystem to the sticky fault string and record to log
    if (problemSubsystem.indexOf(subsystem) == -1) {
      if (problemSubsystem.length() == 0) {
        problemSubsystem = subsystem;
      } else {
        problemSubsystem = StringUtil.buildString(problemSubsystem, ", ", subsystem);
      }
      DataLogUtil.writeMessageEcho(subsystem, "Sticky Faults Logged", "");
    }

    saveStickyFaultsInPrefs(problemSubsystem, problemExists);
    showStickyFaultsOnElastic();
  }

  /**
   * Clears any sticky faults in robotPreferences and Elastic.
   */
  public static void clearStickyFaults() {
    problemSubsystem = "";
    problemExists = false;
    saveStickyFaultsInPrefs(problemSubsystem, problemExists);
    showStickyFaultsOnElastic();
    DataLogUtil.writeMessage("StickyFaults: Sticky Faults Cleared", "");
  }

  /**
   * Shows any sticky faults on Elastic.
   */
  public static void showStickyFaultsOnElastic() {
    SmartDashboard.putString("problemSubsystem", problemSubsystem);
    SmartDashboard.putBoolean("problemExists", problemExists);
  }

  /**
   * Saves a sticky fault in robotPreferences.
   * @param problemSubsystem String name of subsystem in which a problem exists
   * @param problemExists true = has a sticky fault, false = does not have a sticky fault
   */
  private static void saveStickyFaultsInPrefs(String problemSubsystem, boolean problemExists) {
    Preferences.setString("problemSubsystem", problemSubsystem);
    Preferences.setBoolean("problemExists", problemExists);
  }

  /**
   * Returns true if there is currently a sticky fault.
   * @return true = sticky fault exists, false = no faults
   */
  public static boolean isStickyFaultActive() {
    return problemExists;
  }

  /**
   * Verifies that robotPreferences is not empty (which would mean that preferences were accidentally reset).
   * @return true = preferences exist on robot, false = preferences do not exist on robot
   */
  public static boolean prefsExist() {
    return Preferences.containsKey("problemSubsystem");
  }
}