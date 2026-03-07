// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
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
  public static final String bcrRobotCodeVersion = "A1";

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
    public static final int CANTurretEncoder1 = 14;
    public static final int CANTurretEncoder2 = 15;

    public static final int CANPigeonGyro = 22;

    public static final int CANdle = 25;

    public static final int CANIntakeDeployMotor = 30;
    public static final int CANIntakeRollerMotor = 31;

    public static final int CANFeederSpindexerMotor = 32;
    public static final int CANFeederUptakeMotor = 33;

    public static final int CANTurretMotor = 34;

    public static final int CANShooterMotor1 = 35;
    public static final int CANShooterMotor2 = 36;

    public static final int CANClimberMotor1 = 37;
    public static final int CANClimberMotor2 = 38;

    // DIO Ports (add sensors if needed)

    // PWM Ports
    public static final int PWMHoodServo1 = 0;
    public static final int PWMHoodServo2 = 1;
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

  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 1.0000;        // TODO CALIBRATE
    // Left-right distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.25) * DrivetrainAdjustmentFactor;      // CALIBRATED.  FRC2026 CAD = 20.25".
    // Front-back distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.75) * DrivetrainAdjustmentFactor;      // CALIBRATED.  FRC2026 CAD = 22.75".
    
    // Width of robot in meters, + bumpers
    public static final double robotWidth = Units.inchesToMeters(36.5);  // TODO CALIBRATE
    // Length of robot in meters, + bumpers
    public static final double robotLength = Units.inchesToMeters(36.5);  // TODO CALIBRATE
    // Diagonal Width of robot in meters
    public static final double robotDiagonal = Math.sqrt(robotWidth*robotWidth + robotLength*robotLength);
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

    public static final double kMaxSpeedMetersPerSecond = 4.8;      // TODO CALIBRATE
    public static final double kFullSpeedMetersPerSecond = 0.95 * kMaxSpeedMetersPerSecond;
    public static final double kNominalSpeedMetersPerSecond = 0.5 * kMaxSpeedMetersPerSecond;

    // TODO NOT CALIBRATED
    public static final double kMaxAccelerationMetersPerSecondSquare = 6.0;
    public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
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


    public static final double kVDriveAvg = 2.350;     // TODO CALIBRATE.  In voltage per meters/second    2025: LAR was 2.251 (FOC off), AVR was 2.344 (FOC on)
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
    public static final double kADrive = 0.2383;        // TODO CALIBRATE.  In voltage per meters/second^2    2025: LAR was 0.2692 (FOC off), AVR was 0.2383 (FOC on)
    public static final double kADriveToPose = 0.050;   // TODO CALIBRATE.  In time (seconds)
    public static final double kSDrive = 0.141;         // TODO CALIBRATE.  In voltage    2025: LAR was 0.030 (FOC off), AVR was 0.141 (FOC on)

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
  }

  public static class FieldConstants {
    public static final double length = Units.inchesToMeters(651.22);     // CALIBRATED 2026
    public static final double width = Units.inchesToMeters(317.69);      // CALIBRATED 2026
  }

  public static class VisionConstants {
    // TODO CALIBRATE
    // These values are for ETU ~1/31/26
    public static final Transform3d robotToCamUp = new Transform3d(
      new Translation3d(Units.inchesToMeters(-12.125), Units.inchesToMeters(-6.25), Units.inchesToMeters(17.0 + (9.0/16.0))),
      new Rotation3d(0, -0.695, 0));  // in radians!!

    public static final String upCameraName = "UpCamera"; // this has to match the camera name in PhotonVision

    // TODO CALIBRATE (Fake values from PhotonVision example code. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final double kTagDistanceConst = 0.033;
  }
  
  public static final class IntakeConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.5; 
    public static final double reverseIntakePercent = -intakePercent;
  }

  public static final class FeederConstants {
    public static final double compensationVoltage = 12.0;
    public static final double feederGearRatio = 1.0;
    public static final double feedPercent = 0.2; // TODO find a good setpoint if using pctOut
    public static final double kP = 0.45;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.249;
    public static final double kV = 0.082;
    public static final double kA = 0.0;
    public static final double velocityErrorTolerance = 100.0; // TODO tune if necessary

  }
  
  public static final class ShooterConstants {
    public static final double compensationVoltage = 12.0;
    public static final double shooterPercent = 0.2; 
    public static final double reverseShooterPercent = -0.1;

    public static final double gearRatio = 1.0; // Turn ratio from the motor pinion to the shooter wheel

    public static final double ShooterkP = 0.4;    // volts / error in motor rps
    public static final double ShooterkI = 0.0;
    public static final double ShooterkD = 0.0;
    public static final double ShooterkS = 0.1944;   // volts to overcome static friction
    public static final double ShooterkV = 0.1146; // volts / desired motor rps
    public static final double ShooterkA = 0.0;

    public static final double velocityErrorTolerance = 100;
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

  // Colors for the LEDs based on different robot states
  public enum AnimationColor {
    CANDLE_NEUTRAL(0, 0, 0),           // CANdle Black (off)
    CANDLE_STICKY_FAULT(255, 0, 0),    // CANdle Red

    NEUTRAL(0, 0, 0),                  // Black  (strip neutral)
    MATCH_COUNTDOWN(255, 0, 0),        // Red    (strip match countdown)
    INTAKING(255, 255, 0),             // Yellow (strip intaking)
    REVERSE_INTAKING(128, 0, 128),     // Purple (strip reverse intaking)

    DISABLED_BLUE(19, 82, 188),        // Blue   (disabled animation)
    DISABLED_ORANGE(240, 50, 0);       // Orange (disabled animation)

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
      Strip(CANdle.endIndex + 1, (CANdle.endIndex + 1) + 35),
      Full(CANdle.startIndex, Strip.endIndex);

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
      private static final RGBWColor matchCountdownColor = new RGBWColor(AnimationColor.MATCH_COUNTDOWN.r, AnimationColor.MATCH_COUNTDOWN.g, AnimationColor.MATCH_COUNTDOWN.b);
      private static final RGBWColor neutralColor = new RGBWColor(AnimationColor.NEUTRAL.r, AnimationColor.NEUTRAL.g, AnimationColor.NEUTRAL.b);
      private static final RGBWColor intakingColor = new RGBWColor(AnimationColor.INTAKING.r, AnimationColor.INTAKING.g, AnimationColor.INTAKING.b);
      private static final RGBWColor reverseIntakingColor = new RGBWColor(AnimationColor.REVERSE_INTAKING.r, AnimationColor.REVERSE_INTAKING.g, AnimationColor.REVERSE_INTAKING.b);
    }

    public static final class Animations {
      public static final SolidColor CANdleNeutralAnimation = new SolidColor(LEDSegments.CANdle.startIndex, LEDSegments.CANdle.endIndex).withColor(AnimationRGBWColors.CANdleNeutralColor);
      public static final SolidColor CANdleStickyFaultAnimation = new SolidColor(LEDSegments.CANdle.startIndex, LEDSegments.CANdle.endIndex).withColor(AnimationRGBWColors.CANdleStickyFaultColor);

      public static final SingleFadeAnimation disabledBlueAnimation = new SingleFadeAnimation(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.disabledBlueColor).withFrameRate(100);
      public static final SingleFadeAnimation disabledOrangeAnimation = new SingleFadeAnimation(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.disabledOrangeColor).withFrameRate(100);
      public static final ColorFlowAnimation matchCountdownAnimation = new ColorFlowAnimation(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.matchCountdownColor).withFrameRate((LEDSegments.Strip.endIndex - LEDSegments.Strip.startIndex + 1) / 10);
      public static final SolidColor neutralSolidColor = new SolidColor(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.neutralColor);
      public static final StrobeAnimation neutralAnimation = new StrobeAnimation(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.intakingColor);
      public static final StrobeAnimation reverseIntakingAnimation = new StrobeAnimation(LEDSegments.Strip.startIndex, LEDSegments.Strip.endIndex).withColor(AnimationRGBWColors.reverseIntakingColor);
    }
  }
}
