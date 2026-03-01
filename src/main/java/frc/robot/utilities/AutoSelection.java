package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {
	public enum RoutineSelectionOption {
		NONE("None", -1),

		DriveForwardOneMeter("DriveForwardOneMeter", 101),
		DriveForwardTwoMeters("DriveForwardTwoMeters", 102),

		Rel4mRotate180("Relative4m-180", 103),
		RelArcLeft("RelativeArcLeft", 104),
		RelStraight4m("RelativeStraight4m", 105),
		RelCirclePath("RelativeCirclePath", 106),
		RelRotate180("RelativeRotate180", 107),
		AbsDiagonalTest("AbsoluteDiagonalTest", 108);

		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		RoutineSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	public enum StartPositionSelectionOption {
		RIGHT("Right", 0),
		LEFT("Left", 1);

		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		StartPositionSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	private final AllianceSelection allianceSelection;
	private final Field field;
	private final TrajectoryCache trajectoryCache;
	private final Joystick rightJoystick;
	private final DriveTrain driveTrain;

	
	private Command cachedAutonomousCommand;
	private int logRotationKey;
	private int autoPlan;
	private double waitTime;
	private Alliance alliance;

	private SendableChooser<Integer> autoRoutineChooser = new SendableChooser<>();
	// private SendableChooser<Integer> startPositionChooser = new SendableChooser<>();

	/**
	 * Autoselection constructor for command group to set up autoPlan widget.
	 * @param rightJoystick
	 * @param trajectoryCache TrajectoryCache cache
	 * @param allianceSelection AllianceSelection alliance
	 * @param field Field utility
	 * @param driveTrain DriveTrain subsystem
	 */
	public AutoSelection(Joystick rightJoystick, TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field,
						 DriveTrain driveTrain) {
		this.rightJoystick = rightJoystick;
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;
		this.driveTrain = driveTrain;

		logRotationKey = DataLogUtil.allocateLogRotation();

		// Auto selections
		autoRoutineChooser.setDefaultOption(RoutineSelectionOption.NONE.name, RoutineSelectionOption.NONE.value);

		// Set initial values
		autoPlan = -999;
		waitTime = 0;
		alliance = allianceSelection.getAlliance();

		for (RoutineSelectionOption option : RoutineSelectionOption.values()){
			if (option == RoutineSelectionOption.NONE) continue;
			autoRoutineChooser.addOption(option.name, option.value);
		}
	
		// Show auto selection widget on dashboard
		SmartDashboard.putData("Autonomous routine", autoRoutineChooser);

		// Show auto parameters on dashboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		// SmartDashboard.putBoolean("Autonomous use vision", false);

		// Show start position selection widget on dashboard
		// SmartDashboard.putData("Start position", startPositionChooser);

		// Build initial auto command
		buildAutoCommand();
	}

	/**
	 * Builds the auto command based upon input from the shuffleboard and caches it to later be
	 * returned by getAutoCommand().
	 * This method is designed to be called in advance of AutonomousInit by Robot.java.
	 */
	public void buildAutoCommand() {
		Command autonomousCommandMain = null;

		// Get parameters from Shuffleboard
		autoPlan = autoRoutineChooser.getSelected();
		RoutineSelectionOption autoSelectionOption = RoutineSelectionOption.NONE;
		DataLogUtil.writeMessage(true, "AutoSelect, autoPlan =", autoPlan);

		waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15); // make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == RoutineSelectionOption.NONE.value) {
			// Starting position = facing drivers
			autoSelectionOption = RoutineSelectionOption.NONE;
			autonomousCommandMain = new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain);
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardOneMeter.value) {
			autoSelectionOption = RoutineSelectionOption.DriveForwardOneMeter;
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(1, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardTwoMeters.value) {
			autoSelectionOption = RoutineSelectionOption.DriveForwardTwoMeters;
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(2, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.Rel4mRotate180.value) {
			autoSelectionOption = RoutineSelectionOption.Rel4mRotate180;
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.Relative4mRotate180), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelArcLeft.value) {
			autoSelectionOption = RoutineSelectionOption.RelArcLeft;
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeArcLeft), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelStraight4m.value) {
			autoSelectionOption = RoutineSelectionOption.RelStraight4m;
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeStraight4m), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelCirclePath.value) {
			autoSelectionOption = RoutineSelectionOption.RelCirclePath;
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeCirclePath), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelRotate180.value) {
			autoSelectionOption = RoutineSelectionOption.RelRotate180;
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeRotate180), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.AbsDiagonalTest.value) {
			autoSelectionOption = RoutineSelectionOption.AbsDiagonalTest;
			autonomousCommandMain = new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.AbsoluteDiagonalTest), driveTrain, allianceSelection);
		}

		else {
			autoSelectionOption = RoutineSelectionOption.NONE;
			autonomousCommandMain = new WaitCommand(1);
		}

		// Add auto wait time before the main auto command
		cachedAutonomousCommand = new SequentialCommandGroup(
			new WaitCommand(waitTime),
			autonomousCommandMain
		);

		String autoDescription = StringUtil.buildString(allianceSelection.getAlliance(), " ", autoSelectionOption);
		SmartDashboard.putString("Autonomous cached", autoDescription);
		DataLogUtil.writeMessage("AutoSelection: Built ", autoDescription);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * @return the command to run
	 */
	public Command getAutoCommand() {
		if (cachedAutonomousCommand == null) {
			buildAutoCommand();
		}
		DataLogUtil.writeMessage("AutoSelection: GetAutoCommand");
		return cachedAutonomousCommand;
	}

	/**
	 * Schedules the auto command based upon input from the shuffleboard.
	 * This method is designed to be placed on Shuffleboard, so that
	 * the user can run the currently-selected auto command.
	 * @return the command to run
	 */
	public Command scheduleAutoCommand() {
		return new InstantCommand(
		  () -> {
				Command autonomousCommand = getAutoCommand();
				if (autonomousCommand != null) {
					CommandScheduler.getInstance().schedule(autonomousCommand);
				}
		  }
		);
	}

	/**
	 * Runs once per scheduler cycle.
	 */
	public void periodic() {
		if (DataLogUtil.isMyLogRotation(logRotationKey)) {
			int newAutoPlan = autoRoutineChooser.getSelected();
			double newWaitTime = SmartDashboard.getNumber("Autonomous delay", 0);
			Alliance newAlliance = allianceSelection.getAlliance();

			if ( (newAutoPlan != autoPlan) || (newWaitTime != waitTime) ||
				 (newAlliance != alliance)) {
				alliance = newAlliance;
				// Note that buildAutoCommand will update autoPlan and waitTime
				buildAutoCommand();
			}
		}
	}

}