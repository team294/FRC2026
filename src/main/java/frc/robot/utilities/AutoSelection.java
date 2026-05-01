package frc.robot.utilities;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.EndType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;
import frc.robot.commands.autos.*;
import frc.robot.commands.sequences.IntakeSequence;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {
	public enum RoutineSelectionOption {
		// CUSTOM("Custom", -2),
		NONE("None", -1),

		ShootPreload("Center_ShootPreload", 1),
		// Center_ShootPreload_Climb("Center_ShootPreload_Climb", 2),
		Center_CollectUnderTrench("Center_CollectUnderTrench", 3),
		Center_CollectUnderTrench_Climb("Center_CollectUnderTrench_Climb", 4),
		LeftSideFollower("LeftSideFollower", 5),

		// LeftSide_CenterCollect_UnderTrench("LeftSide_CenterCollect_UnderTrench", 1),
		// LeftSide_Preload_CenterCollect_UnderTrench("LeftSide_Preload_CenterCollect_UnderTrench", 2),
		// LeftSide_CenterCollect_UnderTrench_2Cycle("LeftSide_CenterCollect_UnderTrench_2Cycle", 3),
		// // LeftSide_Preload_CenterCollect_UnderTrench_2Cycle("LeftSide_Preload_CenterCollect_UnderTrench_2Cycle", 4),
		// LeftSide_CenterCollect_UnderTrench_Depot("LeftSide_CenterCollect_UnderTrench_Depot", 5),
		// LeftSide_SideCollect_UnderTrench_Depot("LeftSide_Preload_CenterCollect_UnderTrench_Outpost", 6),
		LeftSide_SideCollect_UnderTrench("LeftSide_SideCollect_UnderTrench", 7),
		// LeftSide_SideCollect_OverBump_Depot("LeftSide_Preload_CenterCollect_UnderTrench_Outpost", 8),
		LeftSide_SideCollect_OverBump("LeftSide_SideCollect_OverBump", 9),
		LeftSide_SideCollect_OverBump_2Cycle("LeftSide_SideCollect_OverBump_2Cycle", 10),

		// RightSide_CenterCollect_UnderTrench("RightSide_CenterCollect_UnderTrench", 10),
		RightSide_SideCollect_UnderTrench("RightSide_SideCollect_UnderTrench", 11),
		// RightSide_Preload_CenterCollect_UnderTrench("RightSide_Preload_CenterCollect_UnderTrench", 11),
		// RightSide_CenterCollect_UnderTrench_2Cycle("RightSide_CenterCollect_UnderTrench_2Cycle", 12),
		// RightSide_Preload_CenterCollect_UnderTrench_2Cycle("RightSide_Preload_CenterCollect_UnderTrench_2Cycle", 13),
		// RightSide_CenterCollect_UnderTrench_Outpost("RightSide_CenterCollect_UnderTrench_Outpost", 14),
		// RightSide_SideCollect_UnderTrench_Outpost("RightSide_SideCollect_UnderTrench_Outpost", 15),
		RightSide_SideCollect_OverBump("RightSide_SideCollect_OverBump", 16),
		RightSide_SideCollect_OverBump_2Cycle("RightSide_SideCollect_OverBump_2Cycle", 17);

		// DriveForwardOneMeter("DriveForwardOneMeter", 101),
		// DriveForwardTwoMeters("DriveForwardTwoMeters", 102),

		// Rel4mRotate180("Relative4m-180", 103),
		// RelArcLeft("RelativeArcLeft", 104),
		// RelStraight4m("RelativeStraight4m", 105),
		// RelCirclePath("RelativeCirclePath", 106),
		// RelRotate180("RelativeRotate180", 107),
		// AbsDiagonalTest("AbsoluteDiagonalTest", 108),

		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		RoutineSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	private final AllianceSelection allianceSelection;
	private final Field field;
	private final TrajectoryCache trajectoryCache;
	private final Joystick rightJoystick;
	private final DriveTrain driveTrain;
	private final Intake intake;
	private final Feeder feeder;
	private final Turret turret;
	private final Shooter shooter;
	private final Climber climber;

	private Timer timer = new Timer();
	
	private Command cachedAutonomousCommand;
	private int logRotationKey;
	private int autoPlan;
	private double waitTime;
	private Alliance alliance;

	private SendableChooser<Integer> autoRoutineChooser = new SendableChooser<>();

	/**
	 * Autoselection constructor for command group to set up autoPlan widget.
	 * @param rightJoystick
	 * @param trajectoryCache TrajectoryCache cache
	 * @param allianceSelection AllianceSelection alliance
	 * @param field Field utility
	 * @param driveTrain DriveTrain subsystem
	 */
	public AutoSelection(Joystick rightJoystick, TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field, 
							Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber, DriveTrain driveTrain) {
			    // Intake intake, Feeder feeder, Turret turret, Shooter shooter, Climber climber
		this.rightJoystick = rightJoystick;
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;
		this.driveTrain = driveTrain;
		this.intake = intake;
		this.feeder = feeder;
		this.turret = turret;
		this.shooter = shooter;
		this.climber = climber;

		logRotationKey = DataLogUtil.allocateLogRotation();

		// Auto selections
		autoRoutineChooser.setDefaultOption(RoutineSelectionOption.NONE.name, RoutineSelectionOption.NONE.value);

		// startPositionChooser.setDefaultOption(StartPositionSelectionOption.NONE.name, StartPositionSelectionOption.NONE.value);
		// shootPreloadChooser.setDefaultOption(ShootPreloadSelectionOption.NONE.name, ShootPreloadSelectionOption.NONE.value);
		// ballPickupPathChooser.setDefaultOption(BallPickupPathSelectionOption.NONE.name, BallPickupPathSelectionOption.NONE.value);
		// returnToAllianceChooser.setDefaultOption(ReturnToAllianceSelectionOption.NONE.name, ReturnToAllianceSelectionOption.NONE.value);
		// numCyclesChooser.setDefaultOption(NumCyclesSelectionOption.NONE.name, NumCyclesSelectionOption.NONE.value);
		
		// Set initial values
		autoPlan = -999;
		// startPosition = -999;
		// shootPreload = -999;
		// ballPickup = -999;
		// returnToAlliance = -999;
		// numCycles = -999;
		waitTime = 0;
		alliance = allianceSelection.getAlliance();

		for (RoutineSelectionOption option : RoutineSelectionOption.values()) {
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

		else if (autoPlan == RoutineSelectionOption.ShootPreload.value) {
			// Starting position = facing drivers
			autoSelectionOption = RoutineSelectionOption.ShootPreload;
			autonomousCommandMain = sequence(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 180 : 0, false, driveTrain),
										new ShootPreload(EndType.waitForSetPoint, false, intake, feeder, turret, shooter, climber, driveTrain, field, allianceSelection, trajectoryCache)
									);
									
		}

		// else if (autoPlan == RoutineSelectionOption.Center_ShootPreload_Climb.value) {
		// 	// Starting position = facing drivers
		// 	autoSelectionOption = RoutineSelectionOption.Center_ShootPreload_Climb;
		// 	autonomousCommandMain = new BuildCenterAuton(EndType.waitForSetPoint, false, true, intake, feeder, turret, shooter, climber, driveTrain, field, allianceSelection, trajectoryCache);
		// }

		else if (autoPlan == RoutineSelectionOption.Center_CollectUnderTrench.value) {
			// Starting position = facing drivers
			autoSelectionOption = RoutineSelectionOption.Center_CollectUnderTrench;
			autonomousCommandMain = new BuildCenterAuton(EndType.waitForSetPoint, true, false, intake, feeder, turret, shooter, climber, driveTrain, field, allianceSelection, trajectoryCache);
		}

		else if (autoPlan == RoutineSelectionOption.Center_CollectUnderTrench_Climb.value) {
			// Starting position = facing drivers
			autoSelectionOption = RoutineSelectionOption.Center_CollectUnderTrench_Climb;
			autonomousCommandMain = new BuildCenterAuton(EndType.waitForSetPoint, true, true, intake, feeder, turret, shooter, climber, driveTrain, field, allianceSelection, trajectoryCache);
		}

		// else if (autoPlan == RoutineSelectionOption.DriveForwardOneMeter.value) {
		// 	autoSelectionOption = RoutineSelectionOption.DriveForwardOneMeter;
		// 	autonomousCommandMain = new SequentialCommandGroup(
		// 								new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
		// 								new DriveToPose(CoordType.kRelative, new Pose2d(1, 0, new Rotation2d(0)), driveTrain));
		// }

		// else if (autoPlan == RoutineSelectionOption.DriveForwardTwoMeters.value) {
		// 	autoSelectionOption = RoutineSelectionOption.DriveForwardTwoMeters;
		// 	autonomousCommandMain = new SequentialCommandGroup(
		// 								new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
		// 								new DriveToPose(CoordType.kRelative, new Pose2d(2, 0, new Rotation2d(0)), driveTrain));
		// }

		// else if (autoPlan == RoutineSelectionOption.Rel4mRotate180.value) {
		// 	autoSelectionOption = RoutineSelectionOption.Rel4mRotate180;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.Relative4mRotate180), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelArcLeft.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelArcLeft;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeArcLeft), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelStraight4m.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelStraight4m;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeStraight4m), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelCirclePath.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelCirclePath;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeCirclePath), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelRotate180.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelRotate180;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeRotate180), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.AbsDiagonalTest.value) {
		// 	autoSelectionOption = RoutineSelectionOption.AbsDiagonalTest;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.AbsoluteDiagonalTest), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // climber, 
		// }

		// else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_UnderTrench_Depot.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_UnderTrench_Depot; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kOutpostOrDepot, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // climber, 
		// }

		// else if (autoPlan == RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field);   
		// }

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_UnderTrench.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_UnderTrench; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		// else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_OverBump_Depot.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_OverBump_Depot; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kOutpostOrDepot, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field);   
		// }

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_OverBump.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_OverBump; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kSecondCycleLongerShot, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		else if (autoPlan == RoutineSelectionOption.LeftSideFollower.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSideFollower; 
			autonomousCommandMain = new FollowerAuto(false, EndType.runForever, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		else if (autoPlan == RoutineSelectionOption.RightSide_SideCollect_OverBump.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_SideCollect_OverBump; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kSecondCycleLongerShot, RoutineSideType.kRightSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		else if (autoPlan == RoutineSelectionOption.RightSide_SideCollect_UnderTrench.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_SideCollect_UnderTrench; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kRightSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_OverBump_2Cycle.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_OverBump_2Cycle; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kSecondCycle, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
		}

		else if (autoPlan == RoutineSelectionOption.RightSide_SideCollect_OverBump_2Cycle.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_SideCollect_OverBump_2Cycle; 
			autonomousCommandMain = new BuildSideAuton(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kSecondCycle, RoutineSideType.kRightSide, intake, feeder, turret, shooter, climber, driveTrain, allianceSelection, trajectoryCache, field);   
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
		if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
			int  newAutoPlan = autoRoutineChooser.getSelected();
			// int newStartPosition = startPositionChooser.getSelected(); 
			// int newShootPreload =  shootPreloadChooser.getSelected();
			// int newBallPickup =  ballPickupPathChooser.getSelected();
			// int newReturnToAlliance =  returnToAllianceChooser.getSelected();
			// int newNumCycles =  numCyclesChooser.getSelected();
	
			double newWaitTime = SmartDashboard.getNumber("Autonomous delay", 0);
			Alliance newAlliance = allianceSelection.getAlliance();

			if ( (newAutoPlan != autoPlan) || (newWaitTime != waitTime) ||
				 (newAlliance != alliance) ){
				
				alliance = newAlliance;
				// Note that buildAutoCommand will update autoPlan and waitTime (and other custom selectors)
				buildAutoCommand();
			}
		}
	}

}