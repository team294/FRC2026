// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.DriveConstants.*;

import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveStop;
import frc.robot.utilities.*;

public class DriveTrain extends SubsystemBase implements Loggable {
  
  public enum DriveType {
    kRobotRelative,          // Relative to current robot location/facing
    kDriverRelative,         // Absolute direction, based on location of the Driver (assumes driver is in the current alliance's Driver Station, unless the driver manually resets their position)
    kFieldRelative;          // Absolute field direction, based on location of the current alliance's Driver Station
  }

  private int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private final AllianceSelection allianceSelection;

  // Variables for swerve modules
  private final SwerveModule swerveFrontLeft;
  private final SwerveModule swerveFrontRight;
  private final SwerveModule swerveBackLeft;
  private final SwerveModule swerveBackRight;
    
  // Variables for gyro and gyro calibration
  private final Pigeon2 pigeon = new Pigeon2(CANPigeonGyro, Ports.CANbusMain);
  private final StatusSignal<Angle> pigeonYaw = pigeon.getYaw();
  private final StatusSignal<Angle> pigeonPitch = pigeon.getPitch();
  private final StatusSignal<Angle> pigeonRoll = pigeon.getRoll();
  private final StatusSignal<AngularVelocity> pigeonYawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Boolean> pigeonFault = pigeon.getFault_Hardware();
  private double yawZero = 0.0;
  private double pitchZero = 0.0;

  private final DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> dLogOdometryPose2D = StructLogEntry.create(log, "/DrivePathFollower/curPose2d", Pose2d.struct);
  private final DoubleLogEntry dLogGyroAngle = new DoubleLogEntry(log, "/DriveTrain/GyroAngle");
  private final DoubleLogEntry dLogGyroRaw = new DoubleLogEntry(log, "/DriveTrain/GyroRaw");
  private final DoubleLogEntry dLogGyroVelocity = new DoubleLogEntry(log, "/DriveTrain/GyroVelocity");
  private final DoubleLogEntry dLogGyroPitch = new DoubleLogEntry(log, "/DriveTrain/GyroPitch");
  private final DoubleLogEntry dLogOdometryX = new DoubleLogEntry(log, "/DriveTrain/OdometryX");
  private final DoubleLogEntry dLogOdometryY = new DoubleLogEntry(log, "/DriveTrain/OdometryY");
  private final DoubleLogEntry dLogOdometryTheta = new DoubleLogEntry(log, "/DriveTrain/OdometryTheta");
  private final DoubleLogEntry dLogDriveSpeed = new DoubleLogEntry(log, "/DriveTrain/DriveSpeed");
  private final DoubleLogEntry dLogDriveXVelocity = new DoubleLogEntry(log, "/DriveTrain/DriveXVelocity");
  private final DoubleLogEntry dLogDriveYVelocity = new DoubleLogEntry(log, "/DriveTrain/DriveYVelocity");
  private final DoubleLogEntry dLogBusVoltage = new DoubleLogEntry(log, "/DriveTrain/BusVoltage");
  // Log entries for ChoreoFollower
  private final DoubleLogEntry dLogChoreoBusVoltage = new DoubleLogEntry(log, "/DrivePathFollower/BusVoltage");
  private final DoubleLogEntry dLogChoreoFLDriveOutput = new DoubleLogEntry(log, "/DrivePathFollower/FLDriveOutput");
  private final DoubleLogEntry dLogChoreoFRDriveOutput = new DoubleLogEntry(log, "/DrivePathFollower/FRDriveOutput");
  private final DoubleLogEntry dLogChoreoBLDriveOutput = new DoubleLogEntry(log, "/DrivePathFollower/BLDriveOutput");
  private final DoubleLogEntry dLogChoreoBRDriveOutput = new DoubleLogEntry(log, "/DrivePathFollower/BRDriveOutput");
  
  // Driver angle relative to their driver station.  If the driver is in their alliance's driver station, then this should be zero!
  // This is mainly for use on partial practice fields, where the driver is sitting elsewhere.
  private Rotation2d driverAngleToStation = Rotation2d.kZero;       // Driver's facing relative to their driver station.  (Blue driver relative to Blue station, or Red driver relative to Red station)

  // Variables to store vision cameras
  // TODO (PhotonVision) uncomment when camera(s) are ready on the robot
  // private PhotonCameraWrapper cameraUp;
  List<PhotonCameraWrapper> cameras = new ArrayList<PhotonCameraWrapper>(2);
  private boolean useVisionForOdometry = false;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator; 
  private final Field2d field2d = new Field2d();  // Field to display on Elastic

  private double speedAvg;  // Average speed of the robot chassis

  // Cache a copy of the DriveStop command for using to cancel the default command (typically DriveWithJoystickAdvanced)
  Command driveStopCommand = new DriveStop(this);

  /**
   * Creates the DriveTrain subsystem
   * @param allianceSelection BCR AllianceSelection object
   * @param field BCR Field object to supply AprilTag layout
   */
  public DriveTrain(AllianceSelection allianceSelection, Field field) {
    this.allianceSelection = allianceSelection;
    
    logRotationKey = DataLogUtil.allocateLogRotation();

    // Create and enable cameras
    // TODO (PhotonVision) uncomment when camera(s) are ready on the robot
    // cameraUp = new PhotonCameraWrapper(VisionConstants.robotToCamUp, VisionConstants.upCameraName, field.getAprilTagFieldLayout());
    // cameras.add(cameraUp);

    // Create swerve modules
    swerveFrontLeft = new SwerveModule("FL",
      CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANDriveTurnFrontLeftEncoder, 
      false, true, false, offsetAngleFrontLeftMotor, SwerveConstants.kVmFL);
    swerveFrontRight = new SwerveModule("FR",
      CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANDriveTurnFrontRightEncoder, 
      false, true, false, offsetAngleFrontRightMotor, SwerveConstants.kVmFR);
    swerveBackLeft = new SwerveModule("BL",
      CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANDriveTurnBackLeftEncoder, 
      false, true, false, offsetAngleBackLeftMotor, SwerveConstants.kVmBL);
    swerveBackRight = new SwerveModule("BR",
      CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANDriveTurnBackRightEncoder, 
      false, true, false, offsetAngleBackRightMotor, SwerveConstants.kVmBR);
    
    // Put drive mode on Elastic
    setDriveModeCoast(false);

    pigeonYawVelocity.setUpdateFrequency(100);

    // Zero gyro
    zeroGyro();

    // Create and initialize odometery. Set initial location to (0,0) and facing away from driver (regardless of alliance color)
    Pose2d initialPose = new Pose2d(0, 0, allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
    zeroGyroRotation(initialPose.getRotation().getDegrees());
    poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), getModulePositions(), initialPose);

    // Register notification for when the alliance changes
    this.allianceSelection.addAllianceChangeNotification(this::resetPoseForAlliance);

    // Create field object on Elastic
    SmartDashboard.putData("Field", field2d);

    // Prime the DataLog to reduce delay when first enabling the robot
    updateDriveLog(true);
  }

  // ************ Gyro methods

  /**
   * Verifies if Gyro is still reading.
   * @return true = gryo is connected to Rio, false = not connected
   */
  public boolean isGyroReading() {
    return !pigeonFault.refresh().getValue();
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * @return raw gyro angle, in degrees (positive = left, negative = right)
   */
  private double getGyroRaw() {
    return pigeonYaw.refresh().getValueAsDouble();
  }

  /**
   * Gets the raw gyro pitch.
	 * @return raw gyro pitch, in degrees, from 180 to -180 (postitive = nose up, negative = nose down)
	 */
	private double getGyroPitchRaw() {
		double pitch = pigeonPitch.refresh().getValueAsDouble();
    double roll = pigeonRoll.refresh().getValueAsDouble();

    // TODO apply proper matrix math to account for gyro orientation on the robot
    return pitch;
  }

  /**
   * Zeros the gyro pitch in software to the current pitch.
   */
  public void resetGyroPitch() {
    pitchZero = getGyroPitchRaw();
  }

  /**
	 * Zeros the gyro position in software to the current angle.
	 */
	public void zeroGyro() {
    yawZero = getGyroRaw();
    pitchZero = getGyroPitchRaw();
  }
  
  /**
	 * Zeros the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		yawZero = getGyroRaw() - currentHeading;  // Set yawZero to gyro angle, offset to currentHeading
  }

  /**
   * Returns the current calibrated gyro reading. This is based only upon the gyro angle, and 
   * does not take into account the robot rotation on the field as informed by the AprilTags.
   * <p>In general, most code should use driveTrain.getPoseAngle() instead, which uses information
   * from the AprilTags to help correct the robot rotation.
	 * @return gyro angle, in degrees, from 180 to -180 (postitive = left, negative = right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		angle = MathBCR.normalizeAngle(angle);  // Angle is in raw gyro units (-inf,inf), so it need to be converted to (-180, 180]
		return angle;
  }

  /**
   * Gets the gyro pitch.
	 * @return gyro pitch, in degrees, from 180 to -180 (postitive = nose up, negatives = nose down)
	 */
	public double getGyroPitch() {
		return getGyroPitchRaw() - pitchZero;
  }

  /**
   * Gets the angular velocity of the gyro.
   * @return gyro angular velocity, in degrees per second (positive = turning left, negative = turning right)
   */
  public double getAngularVelocity() {
    return pigeonYawVelocity.refresh().getValueAsDouble();
  }

  // ************ Swerve drive methods

  /**
   * Configures the motors and encoders for every swerve module. The swerve modules are 
   * automatically configured in the SwerveModule constructors. So, this method should not need to be called.
   * <p>However, if the robot browns-out or otherwise partially resets, then this can be used to force 
   * the motors and encoders to have the right calibration and settings, especially the calibration angle 
   * for each swerve module.
   * <p><b>NOTE:</b> This procedure includes multiple blocking calls and will delay robot code.
   */
  public void configureSwerveModules() {
    swerveFrontLeft.configSwerveModule();
    swerveFrontRight.configSwerveModule();
    swerveBackLeft.configSwerveModule();
    swerveBackRight.configSwerveModule();
  }

  /**
   * Sets the drive train to coast or brake mode.
   * <p><b>NORE:</b> This procedure includes multiple blocking calls and will delay robot code by ~250ms 
   * if the mode is changed. However, if setCoast is the same as the current setting, then nothing is 
   * sent to the swerve modules and there will not be a delay.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    swerveFrontLeft.setMotorModeCoast(setCoast);
    swerveFrontRight.setMotorModeCoast(setCoast);
    swerveBackLeft.setMotorModeCoast(setCoast);
    swerveBackRight.setMotorModeCoast(setCoast);

    SmartDashboard.putString("Drive Mode", setCoast ? "Coast" : "Brake");
  }

  /**
   * Gets whether the drive train is in coast mode.
   * @return true = coast mode, false = brake mode
   */
  public boolean isDriveModeCoast() {
    return swerveFrontLeft.isMotorModeCoast() && swerveFrontRight.isMotorModeCoast() && 
      swerveBackLeft.isMotorModeCoast() && swerveBackLeft.isMotorModeCoast();
  }

  /**
   * Sets the drive motors to FOC or trapezoidal commuatation mode.
   * <p><b>NOTE:</b> This takes effect for the <b>next</b> request sent to the motor.
   * @param setFOC true = FOC mode, false = trapezoidal mode
   */
  public void setDriveMotorsFOC(boolean setFOC) {
    swerveFrontLeft.setDriveMotorFOC(setFOC);
    swerveFrontRight.setDriveMotorFOC(setFOC);
    swerveBackLeft.setDriveMotorFOC(setFOC);
    swerveBackRight.setDriveMotorFOC(setFOC);
  }

  /**
   * Sets the percent output of the drive motors.
   * @param percentOutput percent output, -1 to +1
   */
  public void setDriveMotorsOutput(double percentOutput) {
    swerveFrontLeft.setDriveMotorPercentOutput(percentOutput);
    swerveFrontRight.setDriveMotorPercentOutput(percentOutput);
    swerveBackLeft.setDriveMotorPercentOutput(percentOutput);
    swerveBackRight.setDriveMotorPercentOutput(percentOutput);
  }

  /**
   * Sets the percent output of the turning motors.
   * @param percentOutput percent output, -1 to +1
   */
  public void setTurningMotorsOutput(double percentOutput) {
    swerveFrontLeft.setTurnMotorPercentOutput(percentOutput);
    swerveFrontRight.setTurnMotorPercentOutput(percentOutput);
    swerveBackLeft.setTurnMotorPercentOutput(percentOutput);
    swerveBackRight.setTurnMotorPercentOutput(percentOutput);
  }

  /**
   * Sets the facing of the wheels (direction the wheels are pointing).
   * <p><b>NOTE:</b> This sets the absolute facing. It does <b>note</b> turn the wheels to 
   * the "optimized" facing +/-180 degrees if that is closer.
   * @param angle desired wheel facing relative to front of chassis, in degrees, -180 to +180 (positive = left, negative = right, 0 = facing front of robot)
   */
  public void setWheelFacings(double angle) {
    swerveFrontLeft.setWheelFacing(angle);
    swerveFrontRight.setWheelFacing(angle);
    swerveBackLeft.setWheelFacing(angle);
    swerveBackRight.setWheelFacing(angle);
  }

  /**
   * Stops all of the drive and turning motors.
   */
  public void stopMotors() {
    swerveFrontLeft.stopMotors();
    swerveFrontRight.stopMotors();
    swerveBackLeft.stopMotors();
    swerveBackRight.stopMotors();
  }

  /**
   * Sets the states of the swerve modules.
   * @param desiredStates desired swerve module states (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    // Convert states to chassisspeeds and slew limit velocities (limit acceleration) to avoid tipping the robot
    ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates);
    double xSlewed, omegaLimited, ySlewed;
    omegaLimited = chassisSpeeds.omegaRadiansPerSecond; // No limiting
    xSlewed = chassisSpeeds.vxMetersPerSecond;          // No limiting
    ySlewed = chassisSpeeds.vyMetersPerSecond;          // No limiting

    // Discretize the movement to avoid unintended robot translation while robot is rotating
    chassisSpeeds = ChassisSpeeds.discretize(xSlewed, ySlewed, omegaLimited, SwerveConstants.dt);

    // Convert back to swerve module states
    desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d());
    
    // Desaturate wheel speeds to a little below max speed. It takes a while to accelerate to
    // max speed, so reducing the max will help movement accuracy.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    swerveFrontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    swerveFrontRight.setDesiredState(desiredStates[1], isOpenLoop);
    swerveBackLeft.setDesiredState(desiredStates[2], isOpenLoop);
    swerveBackRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  /**
   * Reads the current swerve module states.
   * @return module states, as measured by the encoders (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      swerveFrontLeft.getState(), swerveFrontRight.getState(),
      swerveBackLeft.getState(), swerveBackRight.getState()
    };
  }

  /**
   * Returns the speed of the chassis in X, Y, and theta <b>in the robot frame of reference</b>.
   * <p>Speed of the robot in the x direction, in meters per second (positive = forward)
   * <p>Speed of the robot in the y direction, in meters per second (positive = move to the left)
   * <p>Angular rate of the robot, in radians per second (positive = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Reads the current swerve module positions.
   * @return module positions, as measured by the encoders (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      swerveFrontLeft.getPosition(), swerveFrontRight.getPosition(),
      swerveBackLeft.getPosition(), swerveBackRight.getPosition()
    };
  }
  
  /**
   * Drives the robot using desired robot velocity and orientation, such as from joystick info.
   * @param xSpeed speed of the robot in the x direction, in meters/second (positive = move towards red driver station)
   * @param ySpeed speed of the robot in the y direction, in meters/second (positive = move left in reference to the blue driver station)
   * @param rot angular rate of the robot, in radians/second (positive = turn to the left)
   * @param driveType the provided x and y speeds are relative to one of the following:
   * <p> kRobotRelative:  Relative to current robot location/facing
   * <p> kDriverRelative: Realtive to location of the Driver (assumes driver is in the current alliance's Driver Station, unless the driver manually resets their position)
   * <p> kFieldRelative:  Relative to field, based on location of the current alliance's Driver Station
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void drive(double xSpeed, double ySpeed, double rot, DriveType driveType, boolean isOpenLoop) {
    drive(xSpeed, ySpeed, rot, new Translation2d(), driveType, isOpenLoop);
  }

  /**
   * Drives the robot using desired robot velocity and orientation, such as from joystick info.
   * @param xSpeed speed of the robot in the x direction, in meters/second (positive = move towards red driver station)
   * @param ySpeed speed of the robot in the y direction, in meters/second (positive = move left in reference to the blue driver station)
   * @param rot angular rate of the robot, in radians/second (positive = turn to the left)
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
   *     component, the robot will rotate around that corner.
   * @param driveType the provided x and y speeds are relative to one of the following:
   * <p> kRobotRelative:  Relative to current robot location/facing
   * <p> kDriverRelative: Realtive to location of the Driver (assumes driver is in the current alliance's Driver Station, unless the driver manually resets their position)
   * <p> kFieldRelative:  Relative to field, based on location of the current alliance's Driver Station
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
   public void drive(double xSpeed, double ySpeed, double rot, Translation2d centerOfRotationMeters, DriveType driveType, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeed;

    switch(driveType) {
      default:
      case kRobotRelative:
        chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, rot);
        break;
      case kFieldRelative:
        // TODO Should kFieldRelative use the gyro angle (same as 2025) or the odometry angle (will get updates from camera, which could be wrong)?
        chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()));
        break;
      case kDriverRelative:
        chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()).plus(driverAngleToStation));
        break;
    }

    SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeed, centerOfRotationMeters);
    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  // ************ Driver angle methods

  /**
   * Gets the driver's angle relative to their driver station.  If the driver is in their alliance's driver station, then this should be zero!
   * This is mainly for use on partial practice fields, where the driver is sitting elsewhere.
   * @return driver's angle, in degrees
   */
  public double getDriverAngle() {
    return driverAngleToStation.getDegrees();
  }

  /**
   * Sets the driver's angle relative to their driver station.  This method requires that the robot is facing away from the driver when it is called.
   * This is mainly for use on partial practice fields, where the driver is sitting elsewhere.
   */
  public void resetDriverAngle() {
    driverAngleToStation = Rotation2d.fromDegrees( -getGyroRotation() + (allianceSelection.getAlliance() == Alliance.Blue ? 0 : 180 ));
  }


  // ************ Odometry methods

  /**
   * Gets the currently-estimated position of the robot on the field.
   * @return robot pose on the field. Pose components include:
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the currently-estimated angle of the robot on the field.
   * @return robot angle on the field, in degrees (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public double getPoseAngle() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Resets the pose estimator to the specified pose (defines the robot's position and orientation on the field).
   * This method also resets the gyro, which is required for the pose to properly reset.
   * @param pose pose to set the pose estimator. Pose components include:
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public void resetPose(Pose2d pose) {
    DataLogUtil.writeMessage("Drive.resetPose with set angle =", pose.getRotation().getDegrees(), ", gyro pre =", getGyroRotation(), ", pose pre =", pose.getRotation().getDegrees());
    zeroGyroRotation(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions(), pose);
  }

  /**
   * Resetter used in alliance selection to properly reset the angle when the alliance of the robot changes.
   * @param alliance new alliance of the robot
   */
  public void resetPoseForAlliance(Alliance alliance) {
    Pose2d pose = getPose();

    // When the alliance changes (Blue -> Red or Red -> Blue), rotate the robot orientation by 180 degrees.
    // However, only reset pose if robot is not in the valid field area (such as when robot code boots).
    // Otherwise, the robot is on the field, so don't change anything regardless of the alliance change.
    if (pose.getY() < 0.2 && pose.getX() < 0.2) {
      double rotation = MathBCR.normalizeAngle(pose.getRotation().getDegrees() - 180);
      resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(rotation)));

      // DriveWithJoystickAdvanced will try to turn the robot 180 degrees to maintain the robot heading.
      // Run the DriveStop command to interrupt DriveWithJoystickAdvanced.
      CommandScheduler.getInstance().schedule(driveStopCommand);
    }
  }
  
  /**
   * Returns the speed of the robot in X, Y, and theta <b>in the field frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = away from the Blue drivestation)
   * <p> Speed of the robot in the y direction, in meters per second (+ = left when looking from the Blue drivestation)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(getGyroRotation()));
  }

  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Checks if the CANbus and gyro are working. Sometimes, when the robot boots up, either the CANbus or
   * the gyro don't initialize properly. ROBOT CODE WILL NOT BE ABLE TO CONTROL MOTORS when this happens, so
   * always check this before starting a match!
   * @return true = something is not working, false = CANbus and gyro are both working
   */
  public boolean canBusError() {
    return ((swerveFrontLeft.getDriveBusVoltage() < 7.0) || (swerveFrontLeft.getDriveTemp() < 5.0) || !isGyroReading());
  }


  @Override
  public void periodic() {
    updateOdometry();
    
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateDriveLog(false);

      if(!isGyroReading()) RobotPreferences.recordStickyFaults("Gyro");

      ChassisSpeeds robotSpeeds = getRobotSpeeds();

      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive Speed", speedAvg);
      SmartDashboard.putNumber("Drive X Velocity", robotSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive Y Velocity", robotSpeeds.vyMetersPerSecond);
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putBoolean("Drive isBrakeMode", !isDriveModeCoast());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", getGyroRotation());
      SmartDashboard.putNumber("Drive AngVel", getAngularVelocity());
      SmartDashboard.putNumber("Drive Pitch", getGyroPitch());
      
      // Position from poseEstimator (helpful for auto routines)
      Pose2d pose = poseEstimator.getEstimatedPosition();
      SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
      SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());

      SmartDashboard.putBoolean("Vision Updating Odometry", useVisionForOdometry);

      // TODO This code creates errors after running for a few minutes. Need to debug.
      // SmartDashboard.putData("Swerve Drive", new Sendable() {
      //   @Override
      //   public void initSendable(SendableBuilder builder) {
      //     builder.setSmartDashboardType("SwerveDrive");

      //     builder.addDoubleProperty("Front Left Angle", () -> swerveFrontLeft.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Front Left Velocity", () -> swerveFrontLeft.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Front Right Angle", () -> swerveFrontRight.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Front Right Velocity", () -> swerveFrontRight.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Back Left Angle", () -> swerveBackLeft.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Back Left Velocity", () -> swerveBackLeft.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Back Right Angle", () -> swerveBackRight.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Back Right Velocity", () -> swerveBackRight.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Robot Angle", () -> getGyroRotation()*(Math.PI/180), null);
      //   }
      // });

      // Values from each swerve module
      swerveFrontLeft.updateElastic();
      swerveFrontRight.updateElastic();
      swerveBackLeft.updateElastic();
      swerveBackRight.updateElastic();

      // Values for bug fixing
      SmartDashboard.putNumber("Drive Bus Volt", swerveFrontLeft.getDriveBusVoltage());
    }
  }

  /**
   * Writes information about the driveTrain to the file log.
   * @param logWhenDisabled true = log when enabled or disabled, false = only log when enabled
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    Pose2d pose = getPose();
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    long timeNow = RobotController.getFPGATime();
    
    if (logWhenDisabled  || !DriverStation.isDisabled()) {
      dLogOdometryPose2D.append(pose, timeNow);
      dLogGyroAngle.append(getGyroRotation(), timeNow);
      dLogGyroRaw.append(getGyroRaw(), timeNow);
      dLogGyroVelocity.append(getAngularVelocity(), timeNow);
      dLogGyroPitch.append(getGyroPitch(), timeNow);
      dLogOdometryX.append(pose.getTranslation().getX(), timeNow);
      dLogOdometryY.append(pose.getTranslation().getY(), timeNow);
      dLogOdometryTheta.append(pose.getRotation().getDegrees(), timeNow);
      dLogDriveSpeed.append(speedAvg, timeNow);
      dLogDriveXVelocity.append(robotSpeeds.vxMetersPerSecond, timeNow);
      dLogDriveYVelocity.append(robotSpeeds.vyMetersPerSecond, timeNow);
      dLogBusVoltage.append(swerveFrontLeft.getDriveBusVoltage(), timeNow);

      swerveFrontLeft.updateSwerveLog(timeNow);
      swerveFrontRight.updateSwerveLog(timeNow);
      swerveBackLeft.updateSwerveLog(timeNow);
      swerveBackRight.updateSwerveLog(timeNow);
    }
    
  }

  /**
   * Logs the output percentages of all 4 swerve drive motors and the bus voltage to the datalog.
   * @param timeNow current timestamp for logging
   */
  public void updateDriveLogModuleOutputs(long timeNow) {
    dLogChoreoBusVoltage.append(swerveFrontLeft.getDriveBusVoltage(), timeNow);
    dLogChoreoFLDriveOutput.append(swerveFrontLeft.getDriveOutputPercent(), timeNow);
    dLogChoreoFRDriveOutput.append(swerveFrontRight.getDriveOutputPercent(), timeNow);
    dLogChoreoBLDriveOutput.append(swerveBackLeft.getDriveOutputPercent(), timeNow);
    dLogChoreoBRDriveOutput.append(swerveBackRight.getDriveOutputPercent(), timeNow);
  }

  // ************ Camera methods

  /**
   * Sets whether vision is used in odometry.
   * @param enabled true = uses vision for odometry, false = does not use vision for odometry
   */
  public void setVisionForOdometryState(boolean enabled) {
    useVisionForOdometry = enabled;
  }


  /**
   * Updates odometry based on wheel movement, gyro, and camera/AprilTags.
   */
  public void updateOdometry() {
    // Update odometry based on wheel movement and gyro
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions());

    // Update odometry based on cameras
    for (PhotonCameraWrapper camera : cameras) {
      PhotonPipelineResult camResult = camera.getLatestResult();
      if (camResult != null) {
        Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(), camResult);
        if (result.isPresent()) {
          EstimatedRobotPose camPose = result.get();
          SmartDashboard.putNumber("Vision " + camera.getName() + " X", camPose.estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("Vision " + camera.getName() + " Y", camPose.estimatedPose.toPose2d().getY());
          SmartDashboard.putNumber("Vision " + camera.getName() + " rot", camPose.estimatedPose.toPose2d().getRotation().getDegrees());

          // Only run camera updates for pose estimator if useVisionForOdometry is true
          if (camResult.hasTargets() && useVisionForOdometry) {
            PhotonTrackedTarget bestTarget = camResult.getBestTarget();
            SmartDashboard.putNumber("Vision " + camera.getName() + " AprilTag", bestTarget.getFiducialId());

            // 2025 version:
            // if (bestTarget.getBestCameraToTarget().getX() < 1.5) {
            //   poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, VisionConstants.visionCloseMatrix);
            // } else if (bestTarget.getBestCameraToTarget().getX() < 3) {
            //   poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, VisionConstants.visionFarMatrix);
            // }

            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, 
              camera.getVisionMatrix(result, camResult.getTargets()));

          } else {
            SmartDashboard.putNumber("Vision " + camera.getName() + " AprilTag", -3);
          }
        } else {
          SmartDashboard.putNumber("Vision " + camera.getName() + " AprilTag", -2);
        }
      } else {
        SmartDashboard.putNumber("Vision " + camera.getName() + " AprilTag", -1);
      }
    }
    
    // Update robot average speed
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    speedAvg = Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond);

    // Place robot on field object
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
  }  
}
