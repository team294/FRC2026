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
import frc.robot.Constants.AutoConstants.BallPickupTrajType;
import frc.robot.Constants.AutoConstants.AutoEndType;
import frc.robot.Constants.AutoConstants.ReturnToAllianceType;
import frc.robot.Constants.AutoConstants.RoutineSideType;
import frc.robot.Constants.AutoConstants.ShootPreloadType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;
import frc.robot.commands.autos.*;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {
	public enum RoutineSelectionOption {
		// CUSTOM("Custom", -2),
		NONE("None", -1),

		LeftSide_CenterCollect_UnderTrench("LeftSide_CenterCollect_UnderTrench", 1),
		// LeftSide_Preload_CenterCollect_UnderTrench("LeftSide_Preload_CenterCollect_UnderTrench", 2),
		// LeftSide_CenterCollect_UnderTrench_2Cycle("LeftSide_CenterCollect_UnderTrench_2Cycle", 3),
		// LeftSide_Preload_CenterCollect_UnderTrench_2Cycle("LeftSide_Preload_CenterCollect_UnderTrench_2Cycle", 4),
		LeftSide_CenterCollect_UnderTrench_Depot("LeftSide_CenterCollect_UnderTrench_Depot", 5),
		LeftSide_SideCollect_UnderTrench_Depot("LeftSide_Preload_CenterCollect_UnderTrench_Outpost", 6),
		LeftSide_SideCollect_UnderTrench("LeftSide_SideCollect_UnderTrench", 7),
		LeftSide_SideCollect_OverBump_Depot("LeftSide_Preload_CenterCollect_UnderTrench_Outpost", 8),
		LeftSide_SideCollect_OverBump("LeftSide_SideCollect_UnderTrench", 9),

		RightSide_CenterCollect_UnderTrench("RightSide_CenterCollect_UnderTrench", 10),
		// RightSide_Preload_CenterCollect_UnderTrench("RightSide_Preload_CenterCollect_UnderTrench", 11),
		// RightSide_CenterCollect_UnderTrench_2Cycle("RightSide_CenterCollect_UnderTrench_2Cycle", 12),
		// RightSide_Preload_CenterCollect_UnderTrench_2Cycle("RightSide_Preload_CenterCollect_UnderTrench_2Cycle", 13),
		RightSide_CenterCollect_UnderTrench_Outpost("RightSide_CenterCollect_UnderTrench_Outpost", 14),
		RightSide_SideCollect_UnderTrench_Outpost("RightSide_SideCollect_UnderTrench_Outpost", 15),

		DriveForwardOneMeter("DriveForwardOneMeter", 101),
		DriveForwardTwoMeters("DriveForwardTwoMeters", 102),

		Rel4mRotate180("Relative4m-180", 103),
		RelArcLeft("RelativeArcLeft", 104),
		RelStraight4m("RelativeStraight4m", 105),
		RelCirclePath("RelativeCirclePath", 106),
		RelRotate180("RelativeRotate180", 107),
		AbsDiagonalTest("AbsoluteDiagonalTest", 108),
		DriveOverBump("DriveOverBump", 109);

		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		RoutineSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	// public enum StartPositionSelectionOption {
	// 	NONE("None", null, -1),

	// 	RIGHT("Right", RoutineSideType.kRightSide, 0),
	// 	LEFT("Left", RoutineSideType.kLeftSide, 1);

	// 	@SuppressWarnings({ "MemberName", "PMD.SingularField" })
	// 	public final String name;
	// 	public final int value;
	// 	public final RoutineSideType type;

	// 	StartPositionSelectionOption(String name, RoutineSideType type, int value) {
	// 		this.name = name;
	// 		this.type = type;
	// 		this.value = value;
	// 	}
	// }

	// public enum ShootPreloadSelectionOption {
	// 	NONE("None", null, -1),

	// 	SHOOT_PRELOAD("Shoot Preload", ShootPreloadType.kShootPreload, 0),
	// 	DONT_SHOOT_PRELOAD("Don't Shoot Preload", ShootPreloadType.kDontShootPreload, 1);

	// 	@SuppressWarnings({ "MemberName", "PMD.SingularField" })
	// 	public final String name;
	// 	public final int value;
	// 	public final ShootPreloadType type;

	// 	ShootPreloadSelectionOption(String name, ShootPreloadType type, int value) {
	// 		this.name = name;
	// 		this.type = type;
	// 		this.value = value;
	// 	}
	// }

	// public enum BallPickupPathSelectionOption {
	// 	NONE("None", null, -1),

	// 	CENTER("Pickup Through Center", BallPickupTrajType.kDriveThroughCenter, 0),
	// 	SIDE("Pickup Closer To Hub", BallPickupTrajType.kDriveThroughSide, 1);

	// 	@SuppressWarnings({ "MemberName", "PMD.SingularField" })
	// 	public final String name;
	// 	public final int value;
	// 	public final BallPickupTrajType type;

	// 	BallPickupPathSelectionOption(String name, BallPickupTrajType type, int value) {
	// 		this.name = name;
	// 		this.type = type;
	// 		this.value = value;
	// 	}
	// }

	// public enum ReturnToAllianceSelectionOption {
	// 	NONE("None", null, -1),

	// 	BUMP("Use Bump to Return to Alliance", ReturnToAllianceType.kOverBump, 0),
	// 	TRENCH("Use Trench to Return to Alliance", ReturnToAllianceType.kUnderTrench, 1);

	// 	@SuppressWarnings({ "MemberName", "PMD.SingularField" })
	// 	public final String name;
	// 	public final int value;
	// 	public final ReturnToAllianceType type;

	// 	ReturnToAllianceSelectionOption(String name, ReturnToAllianceType type, int value) {
	// 		this.name = name;
	// 		this.type = type;
	// 		this.value = value;
	// 	}
	// }

	// public enum NumCyclesSelectionOption {
	// 	NONE("None", null, -1),

	// 	ONE("Single cycle to center", AutoEndType.kOne, 0),
	// 	TWO("Two cycles to center", AutoEndType.kTwo, 1);

	// 	@SuppressWarnings({ "MemberName", "PMD.SingularField" })
	// 	public final String name;
	// 	public final int value;
	// 	public final AutoEndType type;

	// 	NumCyclesSelectionOption(String name, AutoEndType type, int value) {
	// 		this.name = name;
	// 		this.type = type;
	// 		this.value = value;
	// 	}
	// }

	private final AllianceSelection allianceSelection;
	private final Field field;
	private final TrajectoryCache trajectoryCache;
	private final Joystick rightJoystick;
	private final DriveTrain driveTrain;
	private final Intake intake;
	private final Feeder feeder;
	private final Turret turret;
	private final Shooter shooter;
	// private final Climber climber;

	
	private Command cachedAutonomousCommand;
	private int logRotationKey;
	private int autoPlan;
	private double waitTime;
	private Alliance alliance;

	// private int startPosition;
	// private int shootPreload;
	// private int ballPickup;
	// private int returnToAlliance;
	// private int numCycles;

	private SendableChooser<Integer> autoRoutineChooser = new SendableChooser<>();

	// private SendableChooser<Integer> startPositionChooser = new SendableChooser<>();
	// private SendableChooser<Integer> shootPreloadChooser = new SendableChooser<>();
	// private SendableChooser<Integer> ballPickupPathChooser = new SendableChooser<>();
	// private SendableChooser<Integer> returnToAllianceChooser = new SendableChooser<>();
	// private SendableChooser<Integer> numCyclesChooser = new SendableChooser<>();

	/**
	 * Autoselection constructor for command group to set up autoPlan widget.
	 * @param rightJoystick
	 * @param trajectoryCache TrajectoryCache cache
	 * @param allianceSelection AllianceSelection alliance
	 * @param field Field utility
	 * @param driveTrain DriveTrain subsystem
	 */
	public AutoSelection(Joystick rightJoystick, TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field, 
							Intake intake, Feeder feeder, Turret turret, Shooter shooter, DriveTrain driveTrain) {
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
		// this.climber = climber;

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

		// for (StartPositionSelectionOption option : StartPositionSelectionOption.values()) {
		// 	if (option == StartPositionSelectionOption.NONE) continue;
		// 	startPositionChooser.addOption(option.name, option.value);
		// }

		// for (ShootPreloadSelectionOption option : ShootPreloadSelectionOption.values()) {
		// 	if (option == ShootPreloadSelectionOption.NONE) continue;
		// 	shootPreloadChooser.addOption(option.name, option.value);
		// }

		// for (BallPickupPathSelectionOption option : BallPickupPathSelectionOption.values()) {
		// 	if (option == BallPickupPathSelectionOption.NONE) continue;
		// 	ballPickupPathChooser.addOption(option.name, option.value);
		// }

		// for (ReturnToAllianceSelectionOption option : ReturnToAllianceSelectionOption.values()) {
		// 	if (option == ReturnToAllianceSelectionOption.NONE) continue;
		// 	returnToAllianceChooser.addOption(option.name, option.value);
		// }

		// for (NumCyclesSelectionOption option : NumCyclesSelectionOption.values()) {
		// 	if (option == NumCyclesSelectionOption.NONE) continue;
		// 	numCyclesChooser.addOption(option.name, option.value);
		// }

		// Show auto selection widget on dashboard
		SmartDashboard.putData("Autonomous routine", autoRoutineChooser);

		// SmartDashboard.putData("Auto Start Position", startPositionChooser);
		// SmartDashboard.putData("Auto Shoot Preload?", shootPreloadChooser);
		// SmartDashboard.putData("Auto Ball Pickup Path", ballPickupPathChooser);
		// SmartDashboard.putData("Auto Return Path", returnToAllianceChooser);
		// SmartDashboard.putData("Auto Number of Cycles", numCyclesChooser);
		

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
		// startPosition = startPositionChooser.getSelected();
		// shootPreload = shootPreloadChooser.getSelected();
		// ballPickup = ballPickupPathChooser.getSelected();
		// returnToAlliance = returnToAllianceChooser.getSelected();
		// numCycles = numCyclesChooser.getSelected();

		
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

		else if (autoPlan == RoutineSelectionOption.DriveOverBump.value) {
			autoSelectionOption = RoutineSelectionOption.DriveOverBump;
			autonomousCommandMain = new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.DriveOverBump), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench; 
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // climber, 
		}

		// else if (autoPlan == RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_UnderTrench.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_UnderTrench; 
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		}

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_OverBump_Depot.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_OverBump_Depot; 
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kOutpostOrDepot, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		}

		else if (autoPlan == RoutineSelectionOption.LeftSide_SideCollect_OverBump.value) {
			autoSelectionOption = RoutineSelectionOption.LeftSide_SideCollect_OverBump; 
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kOverBump, AutoEndType.kNone, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		}

		// else if (autoPlan == RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench_2Cycle.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_CenterCollect_UnderTrench_2Cycle; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kSecondCycle, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		// else if (autoPlan == RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench_2Cycle.value) {
		// 	autoSelectionOption = RoutineSelectionOption.LeftSide_Preload_CenterCollect_UnderTrench_2Cycle; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kSecondCycle, RoutineSideType.kLeftSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		else if (autoPlan == RoutineSelectionOption.RightSide_CenterCollect_UnderTrench.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_CenterCollect_UnderTrench; 
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		}

		// else if (autoPlan == RoutineSelectionOption.RightSide_Preload_CenterCollect_UnderTrench.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RightSide_Preload_CenterCollect_UnderTrench; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kNone, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		// else if (autoPlan == RoutineSelectionOption.RightSide_CenterCollect_UnderTrench_2Cycle.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RightSide_CenterCollect_UnderTrench_2Cycle; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kSecondCycle, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		// else if (autoPlan == RoutineSelectionOption.RightSide_Preload_CenterCollect_UnderTrench_2Cycle.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RightSide_Preload_CenterCollect_UnderTrench_2Cycle; 
		// 	autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kSecondCycle, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field); // intake, feeder, turret, shooter, climber, 
		// }

		else if (autoPlan == RoutineSelectionOption.RightSide_CenterCollect_UnderTrench_Outpost.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_CenterCollect_UnderTrench_Outpost;
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughCenter, ReturnToAllianceType.kUnderTrench, AutoEndType.kOutpostOrDepot, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field);
		}

		else if (autoPlan == RoutineSelectionOption.RightSide_SideCollect_UnderTrench_Outpost.value) {
			autoSelectionOption = RoutineSelectionOption.RightSide_SideCollect_UnderTrench_Outpost;
			autonomousCommandMain = new BuildAutonRoutine(ShootPreloadType.kDontShootPreload, BallPickupTrajType.kDriveThroughSide, ReturnToAllianceType.kUnderTrench, AutoEndType.kOutpostOrDepot, RoutineSideType.kRightSide, intake, feeder, turret, shooter, driveTrain, allianceSelection, trajectoryCache, field);
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