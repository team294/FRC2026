// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.TrapezoidProfileBCR;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String bcrRobotCodeVersion = "A2";

  public enum CoordType {
    kRelative,              // Relative to current robot location/facing
    kAbsolute,              // Absolute field coordinates, don't reset robot pose
    kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
    kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
  }

  // Options for driving stopping types
  public enum StopType {
    kNoStop,
    kCoast,
    kBrake;
  }

  public static enum EndType {
    immediatelyEnd,
    waitForSetPoint,
    runForever
  }

  public enum VelocityType{
    immediatelyEnd,
    runForever, 
    waitForVelocity
  }

  public static final class Ports {
    // Specify which CANbus the drivetrain is on ("rio" or "CANivore")
    public static final String CANbusMainName = "rio";
    public static final CANBus CANbusMain = new CANBus(CANbusMainName);

    public static final String CANbusCANivoreName = "CANivore";
    public static final CANBus CANbusCANivore = new CANBus(CANbusCANivoreName);

    // Define CAN IDs
    public static final int CANDriveFrontLeftMotor = 1;
    public static final int CANDriveFrontRightMotor = 2;
    public static final int CANDriveBackLeftMotor = 3;
    public static final int CANDriveBackRightMotor = 4;

    public static final int CANDriveTurnFrontLeftMotor = 5;
    public static final int CANDriveTurnFrontRightMotor = 6;
    public static final int CANDriveTurnBackLeftMotor = 7;
    public static final int CANDriveTurnBackRightMotor = 8;

    // NOTE: Remote sensors accessed by a Talon FX must have a CAN ID of 15 or less. See errata
    // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
    public static final int CANDriveTurnFrontLeftEncoder = 9;
    public static final int CANDriveTurnFrontRightEncoder = 10;
    public static final int CANDriveTurnBackLeftEncoder = 11;
    public static final int CANDriveTurnBackRightEncoder = 12;
    public static final int CANIntakeDeployEncoder = 13;
    public static final int CANTurretEncoderBelt = 14;
    public static final int CANTurretEncoderGear = 15;

    public static final int CANRangeEntry = 16;
    public static final int CANRangeExit = 17;

    public static final int CANPigeonGyro = 22;

    public static final int CANdle = 25;

    public static final int CANIntakeDeployMotor = 30;
    public static final int CANIntakeRollerMotor = 31;

    public static final int CANFeederSpindexerMotor = 32;
    public static final int CANFeederUptakeMotor = 33;

    public static final int CANTurretMotor = 34;

    public static final int CANShooterMotor1 = 35;
    public static final int CANShooterMotor2 = 36;

    public static final int CANClimberMotorLeft = 37;
    public static final int CANClimberMotorRight = 38;

    public static final int CANHoodMotor1 = 39;
    public static final int CANHoodMotor2 = 40;

    // DIO Ports
    public static final int DIOClimberUpperLimitRightSensor = 0;
    public static final int DIOClimberLowerLimitRightSensor = 1;
    public static final int DIOClimberLowerLimitLeftSensor = 2;
    public static final int DIOClimberUpperLimitLeftSensor = 3;
  }

  public static final class OIConstants {
    public static final int usbXboxController = 0;
    public static final int usbLeftJoystick = 1;
    public static final int usbRightJoystick = 2;
    public static final int usbCoPanel = 3;

    public static final double joystickDeadband = 0.01;
    public static final double joystickJoggingDeadband = 10.0 * joystickDeadband;
    public static final double controllerDeadband = 0.05;
  }

  public static final class ElasticKeyConstants {
    // Commands
    private static final String setPercentCmdString = " Set Percent";
    private static final String setPercentCmdInputString = " Set Percent Input";
    private static final String setVelocityCmdString = " Set Velocity";
    private static final String setVelocityCmdInputString = " Set Velocity Input";
    private static final String setAngleCmdString = " Set Angle";
    private static final String setAngleCmdInputString = " Set Angle Input";
    private static final String setPositionCmdString = " Set Position";
    private static final String setPositionCmdInputString = " Set Position Input";
    private static final String calibrationRampCmdString = " Calibration Ramp";
    private static final String calibrateManualCmdString = " Recalibrate";
    private static final String calibrateManualCmdInputString = " Recalibrate Manual Input";
    private static final String scorePrepCmdString = " Score Prep";
    private static final String stopCmdString = " Stop";

    // Readings
    private static final String rpmString = " RPM";
    private static final String targetRpmString = " Target RPM";
    private static final String tempString = " Temp";
    private static final String angleString = " Angle";
    private static final String targetAngleString = " Target Angle";
    private static final String positionString = " Position";
    private static final String targetPositionString = " Target Position";
    

    // Subsystems
    private static final String intakeString = "Intake";
    private static final String intakeRollerString = "Intake Roller";
    private static final String intakeDeployString = "Intake Deploy";
    private static final String feederSpindexerString = "Spindexer";
    private static final String feederUptakeString = "Uptake";
    private static final String shooterString = "Shooter";
    private static final String hoodString = "Hood";
    private static final String shooterLeftString = shooterString + " L" ; // aka shooter 1
    private static final String shooterRightString = shooterString + " R"; // aka shooter 2
    private static final String hoodLeftString = hoodString + " L";// aka hood 1
    private static final String hoodRightString = hoodString + " R"; // aka hood 2
    private static final String turretString = "Turret";
    private static final String climberString = "Climber";
    private static final String climberLeftString = climberString + " L";
    private static final String climberRightString = climberString + " R";

    public enum ElasticKey {
      STOP_ALL_MOTORS("Stop All Motors"),
      INTAKING_CMD_SEQUENCE("Intake Sequence"),
      PREP_SHOOTER_CMD_SEQUENCE("Prep Shooter Sequence"),
      FEED_SHOOTER_CMD_SEQUENCE("Feed Shooter Sequence"),

      // Stop Command
      INTAKE_CMD_STOP(intakeString + stopCmdString),
      INTAKE_DEPLOY_CMD_STOP(intakeDeployString + stopCmdString),
      INTAKE_ROLLER_CMD_STOP(intakeRollerString + stopCmdString),
      SPINDEXER_CMD_STOP(feederSpindexerString + stopCmdString),
      UPTAKE_CMD_STOP(feederUptakeString + stopCmdString),
      FEEDER_CMD_STOP(feederSpindexerString + "/" + feederUptakeString + stopCmdString),
      SHOOTER_CMD_STOP(shooterString + stopCmdString),
      CLIMBER_CMD_STOP(climberString + stopCmdString),
      CLIMBER_CMD_STOP_LEFT(climberLeftString + stopCmdString),
      CLIMBER_CMD_STOP_RIGHT(climberRightString + stopCmdString),
      HOOD_CMD_STOP_BOTH(hoodString + stopCmdString),
      HOOD_CMD_STOP_LEFT(hoodLeftString + stopCmdString),
      HOOD_CMD_STOP_RIGHT(hoodRightString + stopCmdString),
      TURRET_CMD_STOP(turretString + stopCmdString),

      // Set Voltage Command
      HOOD_CMD_SET_VOLTAGE_LEFT(hoodLeftString + " Set Voltage"),
      HOOD_CMD_SET_VOLTAGE_RIGHT(hoodRightString + " Set Voltage"),
      HOOD_CMD_SET_VOLTAGE_BOTH(hoodString + " Set Voltage (both motors)"),

      // Set Voltage Command Input
      HOOD_CMD_SET_VOLTAGE_INPUT(hoodString + " Set Voltage Input"),

      // Set Percent Command
      INTAKE_ROLLER_CMD_SET_PERCENT(intakeRollerString + setPercentCmdString),
      INTAKE_DEPLOY_CMD_SET_PERCENT(intakeDeployString + setPercentCmdString),
      SPINDEXER_CMD_SET_PERCENT(feederSpindexerString + setPercentCmdString),
      SHOOTER_CMD_SET_PERCENT(shooterString + setPercentCmdString),
      UPTAKE_CMD_SET_PERCENT(feederUptakeString + setPercentCmdString),
      CLIMBER_CMD_SET_PERCENT(climberString + setPercentCmdString),
      CLIMBER_CMD_SET_PERCENT_LEFT(climberLeftString + setPercentCmdString),
      CLIMBER_CMD_SET_PERCENT_RIGHT(climberRightString + setPercentCmdString),
      HOOD_CMD_SET_PERCENT_LEFT(hoodLeftString + setPercentCmdString),
      HOOD_CMD_SET_PERCENT_RIGHT(hoodRightString + setPercentCmdString),
      TURRET_CMD_SET_PERCENT(turretString + setPercentCmdString),

      // Set Percent Command Input
      INTAKE_DEPLOY_CMD_SET_PERCENT_INPUT(intakeDeployString + setPercentCmdInputString),
      INTAKE_ROLLER_CMD_SET_PERCENT_INPUT(intakeRollerString + setPercentCmdInputString),
      SPINDEXER_CMD_SET_PERCENT_INPUT(feederSpindexerString + setPercentCmdInputString),
      SHOOTER_CMD_SET_PERCENT_INPUT(shooterString + setPercentCmdInputString),
      UPTAKE_CMD_SET_PERCENT_INPUT(feederUptakeString + setPercentCmdInputString),
      CLIMBER_CMD_SET_PERCENT_INPUT(climberString + setPercentCmdInputString),
      CLIMBER_CMD_SET_PERCENT_LEFT_INPUT(climberLeftString + setPercentCmdInputString),
      CLIMBER_CMD_SET_PERCENT_RIGHT_INPUT(climberRightString + setPercentCmdInputString),
      HOOD_CMD_SET_PERCENT_INPUT(hoodString + setPercentCmdInputString),
      TURRET_CMD_SET_PERCENT_INPUT(turretString + setPercentCmdInputString),

      // Set Velocity Command
      SPINDEXER_CMD_SET_VELOCITY(feederSpindexerString + setVelocityCmdString),
      UPTAKE_CMD_SET_VELOCITY(feederUptakeString + setVelocityCmdString),
      SHOOTER_CMD_SET_VELOCITY(shooterString + setVelocityCmdString),

      // Set Velocity Command Input
      SPINDEXER_CMD_SET_VELOCITY_INPUT(feederSpindexerString + setVelocityCmdInputString),
      UPTAKE_CMD_SET_VELOCITY_INPUT(feederUptakeString + setVelocityCmdInputString),
      SHOOTER_CMD_SET_VELOCITY_INPUT(shooterString + setVelocityCmdInputString),

      // Set Angle Command
      INTAKE_DEPLOY_CMD_SET_ANGLE(intakeDeployString + setAngleCmdString),
      HOOD_CMD_SET_ANGLE_FOREVER(hoodString + setAngleCmdString + " (Run Forever)"),
      HOOD_CMD_SET_ANGLE_WAIT(hoodString + setAngleCmdString + " (Wait for Angle)"),
      TURRET_CMD_SET_ANGLE(turretString + setAngleCmdString),

      // Set Angle Command Input
      INTAKE_DEPLOY_CMD_SET_ANGLE_INPUT(intakeDeployString + setAngleCmdInputString),
      HOOD_CMD_SET_ANGLE_INPUT(hoodString + setAngleCmdInputString),
      TURRET_CMD_SET_ANGLE_INPUT(turretString + setAngleCmdInputString),

      // Shooter Score Prep Command
      SHOOTER_CMD_SCORE_PREP(shooterString + scorePrepCmdString),

      // Set Position Command
      CLIMBER_CMD_SET_POSITION(climberString + setPositionCmdString),

      // Set Position Command Input
      CLIMBER_CMD_SET_POSITION_INPUT(climberString + setPositionCmdInputString),

      // Calibration Ramp Command
      INTAKE_DEPLOY_CMD_CALIBRATION_RAMP(intakeDeployString + calibrationRampCmdString),
      CLIMBER_CMD_CALIBRATION_RAMP(climberString + calibrationRampCmdString),

      // Calibrate Manual Command
      INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL(intakeDeployString + calibrateManualCmdString),
      CLIMBER_CMD_CALIBRATE_MANUAL(climberString + calibrateManualCmdString),
      TURRET_CMD_RECALIBRATE_MANUAL(turretString + calibrateManualCmdString + " Basic"),
      TURRET_CMD_RECALIBRATE_AUTOMATIC(turretString + calibrateManualCmdString + " Advanced"),

      // Calibrate Manual Command Input
      INTAKE_DEPLOY_CMD_CALIBRATE_MANUAL_INPUT(intakeDeployString + calibrateManualCmdInputString + " (Degrees)"),
      CLIMBER_CMD_CALIBRATE_MANUAL_INPUT(climberString + calibrateManualCmdInputString + " (Inches)"),

      // Voltage
      HOOD_LEFT_VOLTAGE(hoodLeftString + " Voltage"),
      HOOD_RIGHT_VOLTAGE(hoodRightString + " Voltage"),
      HOOD_LEFT_ANALOG_VOLTAGE(hoodLeftString + " Analog Voltage"),
      HOOD_RIGHT_ANALOG_VOLTAGE(hoodRightString + " Analog Voltage"),

      // Current
      HOOD_LEFT_CURRENT(hoodLeftString + " Current"),
      HOOD_RIGHT_CURRENT(hoodRightString + " Current"),
      INTAKE_ROLLER_CURRENT(intakeRollerString + " Current"),

      // Temp
      INTAKE_DEPLOY_TEMP(intakeDeployString + tempString),
      INTAKE_ROLLER_TEMP(intakeRollerString + tempString),
      SPINDEXER_TEMP(feederSpindexerString + tempString),
      UPTAKE_TEMP(feederUptakeString + tempString),
      SHOOTER_LEFT_TEMP(shooterLeftString + tempString),
      SHOOTER_RIGHT_TEMP(shooterRightString + tempString),

      // RPM
      SPINDEXER_RPM(feederSpindexerString + rpmString),
      UPTAKE_RPM(feederUptakeString + rpmString),
      SHOOTER_RPM(shooterString + rpmString),
      INTAKE_ROLLER_RPM(intakeRollerString + rpmString),

      // Target RPM
      SPINDEXER_TARGET_RPM(feederSpindexerString + targetRpmString),
      UPTAKE_TARGET_RPM(feederUptakeString + targetRpmString),
      SHOOTER_TARGET_RPM(shooterString + targetRpmString),

      // Angle
      INTAKE_DEPLOY_ANGLE(intakeDeployString + angleString),
      HOOD_LEFT_ANGLE(hoodLeftString + angleString),
      HOOD_RIGHT_ANGLE(hoodRightString + angleString),
      HOOD_ANGLE(hoodString + angleString),

      // Target Angle
      INTAKE_DEPLOY_TARGET_ANGLE(intakeDeployString + targetAngleString),
      HOOD_ANGLE_CURRENT_SETPOINT(hoodString + targetAngleString),
      TURRET_TARGET_ANGLE(turretString + targetAngleString),

      // Position
      CLIMBER_POSITION(climberString + positionString),
      HOOD_LEFT_LENGTH(hoodLeftString + " Extension Length (in)"),
      HOOD_RIGHT_LENGTH(hoodRightString + " Extension Length (in)"),
      TURRET_ANGLE(turretString + angleString),
      TURRET_ADVANCED_ESTIMATION(turretString + " Advanced Estimation"),
      TURRET_CANCODER_BELT_ABS(turretString + " CANcoder Belt ABS"),
      TURRET_CANCODER_GEAR_ABS(turretString + " CANcoder Gear ABS"),

      // Target Position
      CLIMBER_TARGET(climberString + targetPositionString),

      // CANcoder Connected
      INTAKE_DEPLOY_IS_CANCODER_CONNECTED(intakeDeployString + " CANcoder Connected?"),
      TURRET_IS_CANCODER_CONNECTED(turretString + " CANcoder Connected?"),

      // Using CANcoder
      TURRET_IS_USING_CANCODER(turretString + " Using CANCoder?"),

      // Calibrated
      INTAKE_DEPLOY_IS_CALIBRATED(intakeDeployString + " Calibrated?"),
      CLIMBER_IS_CALIBRATED(climberString + " Calibrated?"),
      TURRET_IS_CALIBRATED(turretString + " Calibrated?"),

      // Raw Rotations
      INTAKE_DEPLOY_RAW_ROTATIONS(intakeDeployString + " Raw Rotations"),
      TURRET_CANCODER_BELT_RAW(turretString + " CANcoder Belt Raw"),
      TURRET_CANCODER_GEAR_RAW(turretString + " CANcoder Gear Raw"),
      TURRET_ENCODER_RAW(turretString + " Encoder Raw"),

      // Limits
      CLIMBER_IS_AT_LOWER_LIMIT(climberString + " Lower Limit"),
      CLIMBER_MOTOR_ONE_IS_AT_LOWER_LIMIT(climberString + " Motor 1 Lower Limit"),
      CLIMBER_MOTOR_TWO_IS_AT_LOWER_LIMIT(climberString + " Motor 2 Lower Limit"),

      // Match
      TELEOP_TIMER("Teleop Timer"),
      SHIFT_TIMER("Shift Timer"),
      IS_BLUE_SHIFT("Is Blue Shift"),
      IS_YOUR_SHIFT("Is Your Shift");

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final String value;
      private ElasticKey(String value) { this.value = value; }
    }
  }

  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 1.0153;        // CALIBRATED = 1.0153
    // Left-right distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.25) * DrivetrainAdjustmentFactor;      // CALIBRATED.  FRC2026 CAD = 20.25".
    // Front-back distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.75) * DrivetrainAdjustmentFactor;      // CALIBRATED.  FRC2026 CAD = 22.75".
    // full length from front bumper to back bumper
    public static final double robotLength = 0; 
    // thickness of front bumper
    public static final double frontBumperThickness = 0; 
    // Width of robot in meters, + bumpers
    public static final double robotWidth = Units.inchesToMeters(13.0);  // TODO CALIBRATE
    // Length of robot in meters, + bumpers
    public static final double robotCenterToFront = Units.inchesToMeters(12);  // TODO CALIBRATE
    public static final double robotCenterToBack = Units.inchesToMeters(16);  // TODO CALIBRATE
    // Diagonal Width of robot in meters
    public static final double robotDiagonal = Math.sqrt(robotWidth*robotWidth + (robotCenterToFront+robotCenterToBack)*(robotCenterToFront+robotCenterToBack));

    // distance from center of robot to corner of intake (in meters):                 (ask Cameron to explain this!!)
    // (half of the robot length) + (intake length) then divide result by cosine of rotation away from wall  
    public static final double robotCenterToIntakeEdge = (robotCenterToFront + IntakeConstants.intakeLength) / Math.cos(FieldConstants.rotationAwayFromWall) - Units.inchesToMeters(3.0);
  }

  public static final class SwerveConstants {
    // Past kDriveGearRatio: FRC2026 = (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0).
    //      Mk4n = 5.90:1 (L2+).  L3 = 6.12:1. Mk4i = 8.14:1 (L1-std gears), 6.75:1 (L2-fast gears), 5.903 (modified L2 16-tooth gear).
    // Past kTurningGearRatio: FRC2026 = 287/11 : 1.  Mk4i = 150 / 7:1.

    // Encoder calibration to meters traveled or wheel facing degrees
    public static final double kEncoderCPR = 1.0;                           // CALIBRATED = 1.0.  Encoder counts per revolution of motor pinion gear
    public static final double kDriveGearRatio = (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0);       // CALIBRATED.  FRC2026 = (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0)
    public static final double kTurningGearRatio = (287.0 / 11.0);                          // CALIBRATED.  FRC2026 = 287/11
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4) * 1.0042;     // CALIBRATED 1.0042.  FRC2026 wheels are nominal 4"
    public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
    public static final double kTurningEncoderDegreesPerTick = 360.0 / kEncoderCPR / kTurningGearRatio;
  
    // Robot calibration for feed-forward and max speeds
    public static final double voltageCompSaturation = 12.0;

    // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
    // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
    // and further derate (initial test by 5%) to account for some battery droop under heavy loads.

    public static final double kMaxSpeedMetersPerSecond = 5.0;      // CALIBRATED
    public static final double kFullSpeedMetersPerSecond = 0.95 * kMaxSpeedMetersPerSecond;
    public static final double kNominalSpeedMetersPerSecond = 0.5 * kMaxSpeedMetersPerSecond;

    public static final double kMaxAccelerationMetersPerSecondSquare = 6.0;       // CALIBRATED 2026.  At slow speeds, robot can accel 20 m/s^2.  At high speeds, battery limits accel to 3 m/s^2
    public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
    // TODO NOT CALIBRATED
    public static final double kNominalAccelerationMetersPerSecondSquare = 3.5;
    public static final double kMaxTurningRadiansPerSecond = 11.0;
    public static final double kNominalTurningRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;     // Not used in code currently
    public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final double driveMotorkP = 0.30;     // kP = (desired-output-volts) / (error-in-encoder-rps)  TODO CALIBRATE FOR 2026
		public static final double driveMotorkI = 0.0;
		public static final double driveMotorkD = 0.000012; // kD = (desired-output-volts) / (error-in-encoder-rps/sec)  TODO CALIBRATE FOR 2026

    public static final double turningMotorkP = 4.3;    // kP = (desired-output-volts) / (error-in-encoder-rotations). CALIBRATED 2026: 4.3
		public static final double turningMotorkI = 0.0;
		public static final double turningMotorkD = 0.1;  // kD = (desired-output-volts) / (error-in-encoder-rps) CALIBRATED 2026: 0.1


    public static final double kVDriveAvg = 2.230;     // CALIBRATED (FOC off, 2.230).  In voltage per meters/second    2025: LAR was 2.251 (FOC off), AVR was 2.344 (FOC on)
    // TODO BELOW NOT CALIBRATED
    private static final double kVmFLrel = 1.0;      // kV modifier for FL drive motor
    private static final double kVmFRrel = 1.0;      // kV modifier for FR drive motor
    private static final double kVmBLrel = 1.0;      // kV modifier for BL drive motor
    private static final double kVmBRrel = 1.0;      // kV modifier for BR drive motor

    // Normalize kVm constants
    private static double kVmAvg = (kVmFLrel + kVmFRrel + kVmBLrel + kVmBRrel) / 4.0;
    public static final double kVmFL = kVmFLrel / kVmAvg;
    public static final double kVmFR = kVmFRrel / kVmAvg;
    public static final double kVmBL = kVmBLrel / kVmAvg;
    public static final double kVmBR = kVmBRrel / kVmAvg;

    public static final double dt = 0.02;               // Timestep for discretizing robot motion, in seconds.  Set this to the scheduler time period = 20ms.
    public static final double kADrive = 0.3112;        // CALIBRATED (FOC off, robot 150xx lbs, 0.3112).  In voltage per meters/second^2    2025: LAR was 0.2692 (FOC off), AVR was 0.2383 (FOC on)
    public static final double kADriveToPose = 0.050;   // CALIBRATED.  In time (seconds)
    public static final double kSDrive = 0.081;         // CALIBRATED (FOC off, 0.081).  In voltage    2025: LAR was 0.030 (FOC off), AVR was 0.141 (FOC on)

    // Minimum abs delta (in m/sec) between actual wheel velocity and desired wheel velocity for kADrive to be applied.
    // If the delta is less than this, then don't use kADrive. This prevents the drive motors from jittering.
    // Note that the swerve module kP in the velocity controller will still work to maintain the proper wheel speed.
    public static final double velMinDeltaUsingkA = 0.3;
  }

  public static final class DriveConstants {
    // The locations of the wheels relative to the physical center of the robot, in meters.
    // X: positive = forward. Y: positive = to the left
    // The order in which you pass in the wheel locations is the same order that you will receive 
    // the module states when performing inverse kinematics. It is also expected that
    // you pass in the module states in the same order when calling the forward kinematics methods.
    // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2)
    );

    // Update the offset angles in RobotPreferences (in Elastic), as well as in this code!
    // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
    // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
    // CALIBRATED 2026
    public static double offsetAngleFrontLeftMotor = -5.712;
    public static double offsetAngleFrontRightMotor = 7.294;
    public static double offsetAngleBackLeftMotor = 0.175;
    public static double offsetAngleBackRightMotor = 151.611;

    // Theta kp value for joystick in rad/sec
    public static final double kPJoystickThetaController = 3;         // TODO CALIBRATE

    public static final double fineControlRatio = 0.4; // CALIBRATED 3/30/26: percent of usual drive speed to drive at during fine control.
  }

  public static class FieldConstants {
    private static final double lengthInches = 651.22; // from field drawings (welded b/c we're in California)
    private static final double widthInches = 317.69;  // from field drawings (welded b/c we're in California)

    public static final double length = Units.inchesToMeters(lengthInches); // x (forward is pos. from BLUE Alliance POV)
    public static final double width = Units.inchesToMeters(widthInches);   // y (left is pos. from BLUE Alliance POV)

    private static final double trenchWidth = 49.84;                       // in inches - distance from side wall to trench (approx.).
    private static final double hubCenterX = 182.11;                       // in inches - distance from driver station wall to hub (x-axis).
    private static final double driverStationWallZone = hubCenterX / 2.0;  // in inches - halfway to center of hub from driver station wall.
    public static final double rotationAwayFromWall = Math.toRadians(45.0);  // in radians. should always be positive.

    public static final double hubCenterToHubEdge = Units.inchesToMeters(23.5);  // in meters - distance from center of hub to side of hub

    public enum FieldBoundaries {
      LEFT_FIELD_WALL(widthInches - trenchWidth, widthInches, hubCenterX, lengthInches - hubCenterX),  // From BLUE Alliance POV
      RIGHT_FIELD_WALL(0, trenchWidth, hubCenterX, lengthInches - hubCenterX),                    // From BLUE Alliance POV
      BLUE_NEUTRAL_ZONE(trenchWidth, widthInches - trenchWidth, hubCenterX, lengthInches / 2.0),
      RED_NEUTRAL_ZONE(trenchWidth, widthInches - trenchWidth, lengthInches / 2.0, lengthInches - hubCenterX),
      BLUE_DRIVER_STATION(0, widthInches, 0, driverStationWallZone),
      RED_DRIVER_STATION(0, widthInches, lengthInches - driverStationWallZone, lengthInches),
      BLUE_ALLIANCE_ZONE(0, widthInches, driverStationWallZone, hubCenterX),
      RED_ALLIANCE_ZONE(0, widthInches, lengthInches - hubCenterX, lengthInches - driverStationWallZone);

      // all from BLUE Alliance POV: yMin is right-most y-axis boundary of the box,
      // yMax is left-most, xMin is closest x-axis boundary of the box to blue
      // alliance driver station, xMax is furthest from blue alliance driver station
      public final double yBoundaryMin, yBoundaryMax, xBoundaryMin, xBoundaryMax;
      private FieldBoundaries(double yMin, double yMax, double xMin, double xMax) {
        this.yBoundaryMin = Units.inchesToMeters(yMin);
        this.yBoundaryMax = Units.inchesToMeters(yMax);
        this.xBoundaryMin = Units.inchesToMeters(xMin);
        this.xBoundaryMax = Units.inchesToMeters(xMax);
      }
    }
    
    public static final Translation2d hubPositionBlue = new Translation2d(Units.inchesToMeters(182.11), width / 2.0);
    public static final Translation2d hubPositionRed = new Translation2d(length - Units.inchesToMeters(182.11), width / 2.0);

    public static final Translation2d leftPassBlue = new Translation2d(Units.inchesToMeters(182.11 / 2.0), width * 3.0 / 4.0);
    public static final Translation2d rightPassBlue = new Translation2d(Units.inchesToMeters(182.11 / 2.0), width / 4.0);
    public static final Translation2d leftPassRed = new Translation2d((length - Units.inchesToMeters(182.11)) / 2.0, width / 4.0);
    public static final Translation2d rightPassRed = new Translation2d((length - Units.inchesToMeters(182.11)) / 2.0, width * 3.0 / 4.0);
  }

  public static class VisionConstants {
    public static final Transform3d robotToCamLeft = new Transform3d(
      new Translation3d(Units.inchesToMeters(-6.534), Units.inchesToMeters(11.910), Units.inchesToMeters(17.080)),  // towards back of robot (-x), left of robot (+y), up (+z)
      new Rotation3d(0.0, Math.toRadians(-23.0), Math.toRadians(180.0 - 16.5))
    );

    public static final Transform3d robotToCamRight = new Transform3d(
      new Translation3d(Units.inchesToMeters(-6.534), Units.inchesToMeters(-11.910), Units.inchesToMeters(17.080)),  // towards back of robot (-x), right of robot (-y), up (+z)
      new Rotation3d(0.0, Math.toRadians(-23.0), Math.toRadians(-(180.0 - 16.5)))
    );

    public static final String leftCameraName = "LeftCamera"; // this has to match the camera name in PhotonVision
    public static final String rightCameraName = "RightCamera"; // this has to match the camera name in PhotonVision

    // TODO CALIBRATE (Fake values from PhotonVision example code. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final double kTagDistanceConst = 0.033;
  }

    public static class AutoConstants {
    // --- enums for building auto routines ---
    /**
     * IMPORTANT: CHANGES START POSITION
     * <p> kShootPreload: starts with the robot's front bumper on start line. Shoots preload, then drives to front of first trajectory
     * <p> kDontShootPreload: starts with robot's back bumper on start line. Immediately starts driving first trajectory
     */
    public enum ShootPreloadType {
      kShootPreload,
      kDontShootPreload
    }

    public enum BallPickupTrajType {
      kDriveThroughCenter, // riskier
      kDriveThroughSide,   // avoid getting our autos messed up if colliding
    }

    /**
     * During autonomous, whether you return to the alliance zone driving over the bump or under the trench.
     */
    public enum ReturnToAllianceType {
      kOverBump,
      kUnderTrench
    }

    /**
     * During autonomous, what to do at the end of the first cycle
     * <p> kNone: do nothing and end at shooting position
     * <p> kSecondCycle: go to the center again for a second cycle
     * <p> kOutpostOrDepot: depending on the side of the auto routine, go to the outpost/depot and intake before shooting balls 
     */
    public enum AutoEndType {
      kNone,
      kSecondCycle,
      kOutpostOrDepot
    }

    /**
     * During autonomous, whether the routine is left or right side, based on the driver perspective. <p>All trajectories are originally on the right side, and are mirrored if it is on the left side.
     */
    public enum RoutineSideType {
      kLeftSide,
      kRightSide
      // center/sideSpecific?
    }
  }

  public static final class IntakeConstants {
    public static final double rollerCompensationVoltage = 12.0;
    public static final double rollerIntakePercent = 1.0;
    public static final double rollerOuttakePercent = -rollerIntakePercent;

    // Length of intake, in meters (from front edge of robot to where intake ends - ignore bumpers)
    public static final double intakeLength = Units.inchesToMeters(12.0);

    public static final double deployGearRatio = (52.0 / 12.0) * (36.0 / 18.0) * (64.0 / 24.0) * (36.0 / 18.0);    // CALIBRATED: (12.0 / 52.0) * (18.0 / 36.0) * (24.0 / 64.0) * (18.0 / 36.0)
    public static final double deployDegreesPerRotation = 360.0;
    public static final double deploySensorToRotorRatio = 36.0 / 18.0;
    public static final double deployCompensationVoltage = 12.0;
    public static final double deployMaxUncalibratedPercentOutput = 0.1;  // TODO calibrate intake
    public static final double deployMaxManualPercentOutput = 0.05;       // TODO calibrate intake
    public static final double deployMaxPercentOutput = 0.2;              // TODO calibrate intake
    public static final double deploySetAngleTolerance = 3.0;             // TODO calibrate intake

    // CANcoder raw angle in degrees when at 0 degrees. This should be updated in RobotPreferences, so it cannot be final.
    public static double deployCANcoderOffsetAngle = -68.2; // 109.64 CALIBRATED: -51.76, 0 degrees is when orange arrow is straight up

    // Center of the region of unallowed motion. 1 makes absolute position unsigned [0, 1), 0.5 makes it signed [-0.5, 0.5), and 0 makes it always negative.
    public static final double deployCANcoderDiscontinuityPoint = -0.7405; // CALIBRATED: -0.7405

    public static final double kP = 30.0;  // CALIBRATED: new 30.0, old intake 10.0 - kP, in Volts/(intake-rotations-error)
    public static final double kI = 0.0;   // CALIBRATED: new 0.0, old intake 0.8 (verified that 0.8 is a very slow correction in angle ~ 10 seconds)
    public static final double kD = 0.2;   // CALIBRATED: new 0.2, old intake 0.4
    public static final double kG = 0.26;  // CALIBRATED: 0.26, in Volts
    public static final double kS = 0.195; // CALIBRATED: new 0.195, old intake 0.185 - kS, in Volts
    public static final double kV = 5.132;  // CALIBRATED: new 5.132, old intake 5.418 - kV, in Volts/(intake-rotations/sec)
    public static final double kA = 0.15;   // CALIBRATED: new 0.15, old intake 0.0 - kV, in Volts/(intake-rotations/sec2)

    public static final double MMCruiseVelocity = 0.8;                   // CALIBRATED, Max velocity in mechanism rotations / second
    public static final double MMAcceleration = MMCruiseVelocity / 0.15;  // CALIBRATED:  new /0.15, old intake /0.35, Max acceleration in mechanism rotations / second^2 (MMVel / MMAccel = sec to full velocity)
    public static final double MMJerk = MMAcceleration / 0.05;            // CALIBRATED, Max jerk in mechanism rotations / second^3 (MMAccel / MMJerk = sec to full acceleration)

    public enum IntakeDeployAngle {
      LOWER_LIMIT(-90.6),   // CALIBRATED 3/29/26: -87.4  Lower limit is intake all the way down, hard stopped by the bumpers
      INTAKE(-90.6),        // CALIBRATED 3/29/26: -87.0
      AGITATE_LOWER(-70.0),  // CALIBRATED 3/19/26: -70.0
      AGITATE_UPPER(-38.0),  // CALIBRATED 3/30/26: -38.0
      START_CONFIG(39.4),  // CALIBRATED 3/29/26: 40.5
      UPPER_LIMIT(39.9);   // CALIBRATED 3/29/26: 41.0   Upper limit is intake all the way in with fuel in the Spindexer

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double value;
      IntakeDeployAngle(double value) { this.value = value; }
    }
  }

  public static final class FeederConstants {
    public static final double reverseSpindexerSeconds = 0.5;
    public static final double noFuelSeconds = 0.687;
    public static final double uptakeVelocityToleranceToStartSpindexer = 50.0;  // in RPM
    public static final double spindexderToUptakeRPMRatio = 0.785;  // TODO CALIBRATE proportion that spindexer should be running at relative to uptake, in RPM. should be < 1.0

    public static final class SpindexerConstants {
      public static final double compensationVoltage = 12.0;
      public static final double spindexerGearRatio = 1.00;
      public static final double spindexerForwardPercent = 0.2; 
      public static final double spindexerReversePercent = -0.2;
      public static final double spindexerShootRPM = 5224.0;  // CALIBRATED 3/27: 5224.0 rpm
      public static final double spindexerReverseRPM = -spindexerShootRPM;  // TODO CALIBRATE

      public static final double kP = 1.2;   // CALIBRATED 3/7/26: 1.2
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.26;  // CALIBRATED 3/7/26: 0.26
      public static final double kV = 0.12;  // CALIBRATED 3/7/26: 0.12
      public static final double kA = 0.0;
    }

    public static final class UptakeConstants {
      public static final double compensationVoltage = 12.0;
      public static final double uptakeGearRatio = 1.0; // doesn't matter b/c we use motor's max output instead of relying on mechanism
      public static final double uptakeForwardPercent = 0.2;
      public static final double uptakeReversePercent = 0.2;
      public static final double uptakeShootRPM = 6650; // CALIBRATED 3/10: 6650

      public static final double kP = 1.4;     // CALIBRATED 3/27: 4.0
      public static final double kI = 0.0;
      public static final double kD = 0.03;    // CALIBRATED 3/27: 0.03
      public static final double kS = 0.25;    // CALIBRATED 3/27: 0.25
      public static final double kV = 0.0936;  // CALIBRATED 3/27: 0.0936
      public static final double kA = 0.0;
    }
  }

  public static final class ShooterConstants {
    public static final double compensationVoltage = 12.0;
    public static final double manualShootPercent = 0.2; 
    public static final double manualReverseShootPercent = -0.1;
    public static final double manualShootRPM = 2500;
    public static final double manualReverseShootRPM = -manualShootRPM;
    public static final double maxShootRPM = 3800;

    public static final double gearRatio = (18.0 / 18.0); // CALIBRATED 3/13/26: (18:18) Turn ratio from the motor pinion to the shooter wheel

    public static final double ShooterkP = 0.35;   // CALIBRATED 3/13: 0.35 - volts / error in motor rps
    public static final double ShooterkI = 0;
    public static final double ShooterkD = 0;
    public static final double ShooterkS = 0.224;  // CALIBRATED 3/13: 0.224 - volts to overcome static friction
    public static final double ShooterkV = 0.118;  // CALIBRATED 3/13: 0.118 volts / desired motor rps
    public static final double ShooterkA = 0;

    public static final double shooterVelocityErrorTolerance = 100;

    // coefficients for (shooter velocity) vs. (distance from hub) functions, used when auto aiming to score
    // Using a linear fit where x is distance (in meters) and returns shooter speed (in rpm)
    public static final double shooterDistanceToRPMIntercept = 1270.76045; // CALIBRATED 3/28/26: 1270.76045
    public static final double shooterDistanceToRPMSlope = 466.41849; // CALIBRATED 3/28/26: 466.41849

    // Hood Constants
    public static final double hoodShootAngle = 30.0; // fixed hood angle to shoot at
    public static final double hoodPassAngle = 35.0;
    public static final double hoodAngleErrorTolerance = 0.7;  // Angle tolerance in degrees

    public static final double hoodVoltage = 12;
    public static final double hoodPIDErrorTolerance = 0.1; // tolerance to pass into the PID controller (deg error)
    public static final double hoodErrorTolerance = 0.2; // to monitor for when to stop moving the hood actuators (deg error)

    public static final double hoodMotorRatio2to1 = 1.10;     // CALIBRATED 3-28-2026.  Ratio of motor power to apply.   (motor1 power) = (motor2 power)*hoodMotorRatio2to1

    public static final double hoodMinimumAngle = 8.637; // in degrees
    //public static final double hoodMaximumAngle = 45.0; // in degrees 
    public static final double hoodMaximumAngle = 40.0; // in degrees Natalie Test

    public static final double hoodMaximumPercentOutput = 0.5;

    public static final double hoodMinimumAnalogVoltage = 0;
    //public static final double hood1MaximumAnalogVoltage = 4.92;
    //public static final double hood2MaximumAnalogVoltage = 4.92;
    public static final double hood1MaximumAnalogVoltage = 4.1; // 3.536
    public static final double hood2MaximumAnalogVoltage = 4.1; // 4.018

    //public static final double hoodActuatorMinExtensionLength = 6.567;   // in inches, measured on physical part (center of bottom circle to center of top circle)
    //public static final double hoodActuatorMaxExtensionLength = 10.4965; // in inches, measured on physical part (center of bottom circle to center of top circle)
    public static final double hoodActuatorMinExtensionLength = 6.567;   // in inches, measured on physical part (center of bottom circle to center of top circle) Natalie Test
    public static final double hoodActuatorMaxExtensionLength = 9.750; // in inches, measured on physical part (center of bottom circle to center of top circle) Natalie Test

    public static final double hood1SlopePointAnalogVoltage = 2.8; // analog voltage, arbitrary value to calculate extension slope 
    public static final double hood2SlopePointAnalogVoltage = 4.04; // analog voltage, arbitrary value to calculate extension slope

    public static final double hood1SlopePointExtensionLength = hoodActuatorMinExtensionLength + 3.172; // in inches, arbitrary extension length corresponding to hood1SlopePointAnalogVoltage
    public static final double hood2SlopePointExtensionLength = hoodActuatorMinExtensionLength + 3.25; // in inches, arbitrary extension length corresponding to hood2SlopePointAnalogVoltage

    public static final double hoodToleranceFromTopOrBottom = 0.005;

    public static final double hoodMinimumAnalogVoltageCutoff = hoodMinimumAnalogVoltage + hoodToleranceFromTopOrBottom;
    public static final double hood1MaximumAnalogVoltageCutoff = hood1MaximumAnalogVoltage - hoodToleranceFromTopOrBottom;
    public static final double hood2MaximumAnalogVoltageCutoff = hood2MaximumAnalogVoltage - hoodToleranceFromTopOrBottom;

    public static final double radius1 = 7.678; // the length of one side of the triangle formed with the linear actuator for the hood angle
    public static final double radius2 = 6.361; // the length of the other side of the triangle 

    public static final double hood1VoltageToExtensionLengthSlope = (hood1SlopePointExtensionLength - hoodActuatorMinExtensionLength) / (hood1SlopePointAnalogVoltage - hoodMinimumAnalogVoltage);
    public static final double hood2VoltageToExtensionLengthSlope = (hood2SlopePointExtensionLength - hoodActuatorMinExtensionLength) / (hood2SlopePointAnalogVoltage - hoodMinimumAnalogVoltage);

    public static final double hood1kP = 0.26;   //was 0.24 // volts / error in motor rps
    public static final double hood1kI = 0.0;
    public static final double hood1kD = 0.0;
    public static final double hood1kS = 0.0;   // voltage output to overcome static friction (constant added to basic PID control to overcome static friction from having to move the hood)
    public static final double hood1kV = 0.0; // volts / desired motor rps
    public static final double hood1kA = 0.0;
    public static final double hood2kP = 0.275;    // volts / error in motor rps
    public static final double hood2kI = 0.0;
    public static final double hood2kD = 0.0;
    public static final double hood2kS = 0.0;   // voltage output to overcome static friction (constant added to basic PID control to overcome static friction from having to move the hood)
    public static final double hood2kV = 0.0; // volts / desired motor rps
    public static final double hood2kA = 0.0;

    public static final double hoodMaxVelocity = 1.0;
    public static final double hoodMaxAcceleration = 1.0;
  }

  public static final class TurretConstants {
    public static final double compensationVoltage = 12.0;

    public static final double turretToMotorGearRatio = (70.0 / 12.0) * (135.0 / 15.0);  // CALIBRATED: (70.0 / 12.0) * (135.0 / 15.0) For gear reduction from motor to turret, value should be greater than 1
    public static final double turretBeltTeeth = 135.0;
    public static final double beltEncoderBeltTeeth = 24.0;
    public static final double beltEncoderGearTeeth = 30.0;
    public static final double gearEncoderGearTeeth = 26.0;

    public static final double gearEncoderToTurretRatio = TurretConstants.beltEncoderBeltTeeth / TurretConstants.turretBeltTeeth;
    public static final double encoderCalculationCoefficient = gearEncoderToTurretRatio  /  (beltEncoderGearTeeth / gearEncoderGearTeeth - 1);
    public static final double turretDegreesPerRotation = 360.0;  // Turret degrees per rotation of the motor sensor.  The gear ratio is calculated in the motor configurator, so don't apply the gear ratio here.

    public static final double offsetAngleCANcoderBelt = 0.1052;  // CALIBRATED: 0.1052 - (1) set this value to 0.0.  (2) Read value of belt encoder raw rotations when turret is at absolute zero (facing back of robot), and place that reading here.
    public static final double offsetAngleCANcoderGear = -0.4818; // CALIBRATED: -0.4818 (1) set this value to 0.0.  (2) Read value of gear encoder raw rotations when turret is at absolute zero (facing back of robot), and place that reading here.

    public static final double maxPercentOutput = 0.3;               // Absolute max percent out allowed for turret
    public static final double maxManualPercentOutput = 0.2; // Max percent out allowed for manual turret movement when calibrated
    public static final double maxUncalibratedPercentOutput = 0.05;   // Max percent out allowed for any turrent movement when uncalibrated

    public static final double angleTolerance = 1.0;        // Angle tolerance for setting the turret angle, in degrees

    public static final double kP = 90.0;  // CALIBRATED 3/17/26: 90.0
    public static final double kI = 0.0;   // CALIBRATED 3/17/26: 0.0
    public static final double kD = 0.03;   // CALIBRATED 3/17/26: 0.0
    public static final double kS = 0.275; // CALIBRATED 3/17/26: 0.275
    public static final double kV = 5.1;   // CALIBRATED 3/17/26: 5.1
    public static final double kA = 0.0;   // CALIBRATED 3/17/26: 0.0

    public static final double MMCruiseVelocity = 20.0; // CALIBRATED 3/17/26: 20.0
    public static final double MMAcceleration = 9.0;    // CALIBRATED 3/17/26: 9.0
    public static final double MMJerk = 0.0;           // CALIBRATED 3/28/26: 0.0

    public static final double turretOffset = Units.inchesToMeters(-11.086); // Calibrated: 11.086 inches - straight line offset from center of robot to center of turret (in meters)
    
    public enum TurretAngle {
      LOWER_LIMIT(-180.0),
      UPPER_LIMIT(180.0);

      public final double value;
      TurretAngle(double angle) {
        this.value = angle;
      }
    }
  }

  public static final class ClimberConstants {
    // Side convention for all climber side-specific commands/logic:
    // 1 = left, 2 = right
    public static final int SIDE_LEFT = 1;
    public static final int SIDE_RIGHT = 2;

    public static final double compensationVoltage = 12.0;

    public static final double maxManualPercentOutput = 0.3; // TODO CALIBRATE
  }

  public static final class TrajectoryConstants {
    // Max error for robot rotation
    public static final double maxThetaErrorDegrees = 1.0;
    public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

    // Max error for interim positions (not final)
    public static final double interimThetaErrorDegrees = 2.0;        
    public static final double interimPositionErrorMeters = 0.20; // 8 inches

    public static final double xError = 0.25;
    public static final double yError = 0.25;
    public static final double choreoMaxThetaErrorDegrees = 5;
    public static final double endVelocityErrorDegrees = 0.75;

    // Feedback terms for holonomic drive controllers

    // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
    public static final double kPXController = 1;

    // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)  
    public static final double kPYController = 1; 

    // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)
    public static final double kPThetaController = 3;

    public static final TrajectoryConfig swerveTrajectoryConfig =
          new TrajectoryConfig(
                  SwerveConstants.kNominalSpeedMetersPerSecond,
                  SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
              .setKinematics(DriveConstants.kDriveKinematics);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
          SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

    // Constraint for the DriveToPose motion profile for distance being travelled
    public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
    new TrapezoidProfileBCR.Constraints(
          SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
  }

  public static final class CANRangeConstants {
    public static final double proximityThreshold = 0.15; // CALIBRATED 3/10: 0.15
    public static final double maxDistanceError = 0.01;
  }

  // Colors for the LEDs based on different robot states
  public enum AnimationColor {
    CANDLE_NEUTRAL(0, 0, 0),           // CANdle Black (off)
    CANDLE_STICKY_FAULT(255, 0, 0),    // CANdle Red

    NEUTRAL(0, 0, 0),                  // Black  (strip neutral)
    INTAKING(255, 255, 0),             // Yellow (strip intaking)
    REVERSE_INTAKING(128, 0, 128),     // Purple (strip reverse intaking)
    PREP_SHOOTER(255, 50, 0),          // Orange (strip prep shooter)
    SHOOTER_READY(19, 82, 188),         // Blue (strip shooter ready)
    FEEDING_SHOOTER(19, 82, 188),       // Blue (feeding shooter)

    LOWERING(255, 0, 0),               // Red (lowering climber and hood)
    TRENCH_SAFE(0, 255, 0),            // Green (climber and hood at bottom, trench safe)

    DISABLED_BLUE(19, 82, 188),        // Blue   (disabled animation)
    DISABLED_ORANGE(255, 50, 0);       // Orange (disabled animation)

    public final int r, g, b;
    AnimationColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public static final class LEDConstants {
    public enum LEDSegments {
      CANdle(0, 7),
      Strip1(CANdle.endIndex + 1, (CANdle.endIndex + 1) + 14),
      Strip2(Strip1.endIndex + 1, (Strip1.endIndex + 1) + 14),
      Strip3(Strip2.endIndex + 1, (Strip2.endIndex + 1) + 14),
      StripAll(Strip1.startIndex, Strip3.endIndex),
      Full(CANdle.startIndex, Strip3.endIndex);

      public final int startIndex, endIndex;
      LEDSegments(int index, int count) {
        this.startIndex = index;
        this.endIndex = count;
      }
    }

    private final class AnimationRGBWColors {
      private static final RGBWColor CANdleNeutralColor = new RGBWColor(AnimationColor.CANDLE_NEUTRAL.r, AnimationColor.CANDLE_NEUTRAL.g, AnimationColor.CANDLE_NEUTRAL.b);
      private static final RGBWColor CANdleStickyFaultColor = new RGBWColor(AnimationColor.CANDLE_STICKY_FAULT.r, AnimationColor.CANDLE_STICKY_FAULT.g, AnimationColor.CANDLE_STICKY_FAULT.b);

      private static final RGBWColor disabledBlueColor = new RGBWColor(AnimationColor.DISABLED_BLUE.r, AnimationColor.DISABLED_BLUE.g, AnimationColor.DISABLED_BLUE.b);
      private static final RGBWColor disabledOrangeColor = new RGBWColor(AnimationColor.DISABLED_ORANGE.r, AnimationColor.DISABLED_ORANGE.g, AnimationColor.DISABLED_ORANGE.b);
      private static final RGBWColor neutralColor = new RGBWColor(AnimationColor.NEUTRAL.r, AnimationColor.NEUTRAL.g, AnimationColor.NEUTRAL.b);
      private static final RGBWColor intakingColor = new RGBWColor(AnimationColor.INTAKING.r, AnimationColor.INTAKING.g, AnimationColor.INTAKING.b);
      private static final RGBWColor reverseIntakingColor = new RGBWColor(AnimationColor.REVERSE_INTAKING.r, AnimationColor.REVERSE_INTAKING.g, AnimationColor.REVERSE_INTAKING.b);
      private static final RGBWColor shooterPrepColor = new RGBWColor(AnimationColor.PREP_SHOOTER.r, AnimationColor.PREP_SHOOTER.g, AnimationColor.PREP_SHOOTER.b);
      private static final RGBWColor shooterReadyColor = new RGBWColor(AnimationColor.SHOOTER_READY.r, AnimationColor.SHOOTER_READY.g, AnimationColor.SHOOTER_READY.b);
      private static final RGBWColor feedingShooterColor = new RGBWColor(AnimationColor.SHOOTER_READY.r, AnimationColor.SHOOTER_READY.g, AnimationColor.SHOOTER_READY.b);

      private static final RGBWColor trenchSafeColor = new RGBWColor(AnimationColor.TRENCH_SAFE.r, AnimationColor.TRENCH_SAFE.g, AnimationColor.TRENCH_SAFE.b);
    }

    public static final class Animations {
      public static final SolidColor CANdleNeutralAnimation = new SolidColor(LEDSegments.CANdle.startIndex, LEDSegments.CANdle.endIndex).withColor(AnimationRGBWColors.CANdleNeutralColor);
      public static final SolidColor CANdleStickyFaultAnimation = new SolidColor(LEDSegments.CANdle.startIndex, LEDSegments.CANdle.endIndex).withColor(AnimationRGBWColors.CANdleStickyFaultColor);

      public static final TwinkleAnimation disabledBlueAnimation1 = new TwinkleAnimation(LEDSegments.Strip1.startIndex, LEDSegments.Strip1.endIndex).withColor(AnimationRGBWColors.disabledBlueColor).withSlot(1).withMaxLEDsOnProportion(0.7);
      public static final TwinkleAnimation disabledBlueAnimation2 = new TwinkleAnimation(LEDSegments.Strip2.startIndex, LEDSegments.Strip2.endIndex).withColor(AnimationRGBWColors.disabledBlueColor).withSlot(2).withMaxLEDsOnProportion(0.7);
      public static final TwinkleAnimation disabledBlueAnimation3 = new TwinkleAnimation(LEDSegments.Strip3.startIndex, LEDSegments.Strip3.endIndex).withColor(AnimationRGBWColors.disabledBlueColor).withSlot(3).withMaxLEDsOnProportion(0.7);
      public static final TwinkleAnimation disabledOrangeAnimation1 = new TwinkleAnimation(LEDSegments.Strip1.startIndex, LEDSegments.Strip1.endIndex).withColor(AnimationRGBWColors.disabledOrangeColor).withSlot(1).withMaxLEDsOnProportion(0.7);
      public static final TwinkleAnimation disabledOrangeAnimation2 = new TwinkleAnimation(LEDSegments.Strip2.startIndex, LEDSegments.Strip2.endIndex).withColor(AnimationRGBWColors.disabledOrangeColor).withSlot(2).withMaxLEDsOnProportion(0.7);
      public static final TwinkleAnimation disabledOrangeAnimation3 = new TwinkleAnimation(LEDSegments.Strip3.startIndex, LEDSegments.Strip3.endIndex).withColor(AnimationRGBWColors.disabledOrangeColor).withSlot(3).withMaxLEDsOnProportion(0.7);
      public static final SolidColor neutralSolidColor = new SolidColor(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.neutralColor);
      public static final StrobeAnimation neutralAnimation = new StrobeAnimation(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.intakingColor);
      public static final StrobeAnimation reverseIntakingAnimation = new StrobeAnimation(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.reverseIntakingColor);
      public static final StrobeAnimation shooterPrepAnimation = new StrobeAnimation(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.shooterPrepColor);
      public static final SolidColor shooterReadySolidColor = new SolidColor(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.shooterReadyColor);
      public static final StrobeAnimation feedingShooterAnimation = new StrobeAnimation(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.feedingShooterColor);

      public static final RainbowAnimation climberMovingAnimation = new RainbowAnimation(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex);
      public static final SolidColor trenchSafeSolidColor = new SolidColor(LEDSegments.StripAll.startIndex, LEDSegments.StripAll.endIndex).withColor(AnimationRGBWColors.trenchSafeColor);
    }
  }
}
