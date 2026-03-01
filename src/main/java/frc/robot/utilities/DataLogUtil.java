/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import static frc.robot.utilities.StringUtil.*;

/**
 * A utility to write information to a file for loggin.
 */
public class DataLogUtil {
	// Level of detail between 1-3, where 1 is the most detailed and 3 is the least detailed
	private static int logLevel = 3;
    
  	// File logging rotation cycles, to spread out logging times between subsystems	
	private static final int NUM_ROTATIONS = 10;
	private static int rotationLastAllocated = NUM_ROTATIONS - 1;
	private static int rotationCurrent = 0;	// Values = 0 ... NUM_ROTATIONS - 1

	// DataLog entry to use for logging messages and events
    private static final StringLogEntry dLogMessage = new StringLogEntry(DataLogManager.getLog(), "/DataLog/message");


	/** Start data log manager with default directory location. */
	public static void start() {
	  DataLogManager.start();
	}

	/**
	 * Writes a message to the log file. The message will be timestamped. Does not echo the message to the screen.
	 * If more than one string or object is provided, then places one space between each item.
	 * @param paramArray... List of descriptions and values (variable number of parameters)
	 */
	public static void writeMessage(Object... paramArray) {
		// Write the message to the file
		dLogMessage.append(buildStringWithDelimiter(" ", buildString((Object [])paramArray)));
	}

	/**
	 * Writes a message to the log file. The message will be timestamped. The message will be echoed to the screen.
	 * If more than one string or object is provided, then places one space between each item.
	 * @param paramArray... List of descriptions and values (variable number of parameters)
	 */
	public static void writeMessageEcho(Object... paramArray) {
		String s = buildStringWithDelimiter(" ", (Object [])paramArray);
		writeMessage(s);
		System.out.println(buildString("Log:  ", s));
	}

  	/**
	 * Changes level of detail for the fileLog.
	 * Level 1 = full debugging logs. Huge file log, so use sparingly.
	 * Level 2 = normal lab mode. Moderate logging details.
	 * Level 3 = competition mode. Minimal logging.
	 * @param level detail level between 1-3, where 1 is the most detailed and 3 is the least detailed
	 */
	public static void setLogLevel(int level) {
		logLevel = level;
		writeMessageEcho("DataLogUtil:  setLogLevel, Level =", level);
	}

	/**
	 * Returns what level of detail the fileLog should be at, to be called in each subsystem.
	 * Level 1 = full debugging logs. Huge file log, so use sparingly.
	 * Level 2 = normal lab mode. Moderate logging details.
	 * Level 3 = competition mode. Minimal logging.
	 */
	public static int getLogLevel() {
		return logLevel;
	}

	/** 
	 * Advances the log rotation counter by one place, and resets if above threshhold for the current log level.
	 */
	public static void advanceLogRotation() {
		rotationCurrent++;
		if (rotationCurrent >= NUM_ROTATIONS) rotationCurrent = 0;

		// if (logLevel == 3) {
		// 	if (rotation >= 25) rotation = 0;
		// } else {
		// 	if (rotation >= 10) rotation = 0;

		// TODO add log levels if we choose to implement
	}

	/**
	 * Gets the index of the file log rotation.
	 * @return int between 0 and 1 (log levels 1 or 2) or between 0 and 24 (log level 3)
	 */
	public static int getLogRotation() {
		return rotationCurrent;
	}

	/**
	 * Allocates a rotation index to a subsystem. Use the returned value (logRotationKey) with
	 * isMyLogRotation(logRotationKey) to see if this is the current rotation for a given subsystem.
	 * @return allocated index for this subsystem
	 */
	public static int allocateLogRotation() {
		rotationLastAllocated++;
		rotationLastAllocated %= NUM_ROTATIONS;
		return rotationLastAllocated;
	}

	/**
	 * Returns true if the scheduler is currently at the rotationKey.
	 * @param logRotationKey Key from allocateLogRotation() to check
	 * @return true = rotationKey is the current rotation, false = rotationKey is not the current rotation
	 */
	public static boolean isMyLogRotation(int logRotationKey) {
		return (logRotationKey == rotationCurrent);
	}

}