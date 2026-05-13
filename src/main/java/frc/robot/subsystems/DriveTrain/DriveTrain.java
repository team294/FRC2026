// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.commands.DriveStop;
import frc.robot.subsystems.PhotonCameraWrapper;
import frc.robot.utilities.*;
import frc.robot.utilities.DataLogUtil.RotationWhenDisabled;

import static frc.robot.Constants.DriveConstants.*;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class DriveTrain extends SubsystemBase implements Loggable {
    public enum DriveType {
        kRobotRelative,   // Relative to current robot location/facing
        kDriverRelative,  // Absolute direction, based on location of the Driver
        kFieldRelative;   // Absolute field direction, based on location of the current alliance's Driver Station
    }

    private int logRotationKey;
    private boolean fastLogging = false;
    private boolean recordedStickyFault = false;
    private final AllianceSelection allianceSelection;

    // AdvantageKit IO layers
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModule[] swerveModules; // FL=0, FR=1, BL=2, BR=3

    // Gyro software offsets
    private double yawZero = 0.0;
    private double pitchZero = 0.0;

    // Vision
    private PhotonCameraWrapper cameraLeft;
    private PhotonCameraWrapper cameraRight;
    private final List<PhotonCameraWrapper> cameras = new ArrayList<>(2);
    private boolean useVisionForOdometry = true;

    // Odometry / pose
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private double speedAvg;

    // Locking axis movement / fine control
    private boolean xLocked = false;
    private boolean yLocked = false;
    private boolean turnLocked = false;
    private boolean fineControl = false;

    // Driver angle relative to their driver station
    private Rotation2d driverAngleToStation = Rotation2d.kZero;

    // Cache DriveStop command for interrupting the default command
    Command driveStopCommand = new DriveStop(this);

    /**
     * Creates the Drive subsystem with AdvantageKit IO layers.
     *
     * @param gyroIO            GyroIO implementation (real or sim)
     * @param flModuleIO        Front Left SwerveModuleIO
     * @param frModuleIO        Front Right SwerveModuleIO
     * @param blModuleIO        Back Left SwerveModuleIO
     * @param brModuleIO        Back Right SwerveModuleIO
     * @param allianceSelection AllianceSelection utility
     * @param field             Field utility to supply AprilTag layout
     */
    public DriveTrain(GyroIO gyroIO, SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO,
                SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO,
                AllianceSelection allianceSelection, Field field) {

        this.gyroIO = gyroIO;
        this.allianceSelection = allianceSelection;

        logRotationKey = DataLogUtil.allocateLogRotation();

        swerveModules = new SwerveModule[]{
            new SwerveModule(flModuleIO, 0, SwerveConstants.kVmFL),
            new SwerveModule(frModuleIO, 1, SwerveConstants.kVmFR),
            new SwerveModule(blModuleIO, 2, SwerveConstants.kVmBL),
            new SwerveModule(brModuleIO, 3, SwerveConstants.kVmBR)
        };

        // Create and enable cameras
        cameraLeft = new PhotonCameraWrapper(VisionConstants.robotToCamLeft,  VisionConstants.leftCameraName,  field.getAprilTagFieldLayout());
        cameraRight = new PhotonCameraWrapper(VisionConstants.robotToCamRight, VisionConstants.rightCameraName, field.getAprilTagFieldLayout());
        cameras.add(cameraLeft);
        cameras.add(cameraRight);

        // Put Field2d on Elastic dashboard
        SmartDashboard.putData("Field", field2d);

        // Initialize drive mode and gyro
        setDriveModeCoast(false);
        zeroGyro();

        // Initialize pose estimator
        Pose2d initialPose = new Pose2d(0, 0,
            allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
        zeroGyroRotation(initialPose.getRotation().getDegrees());
        poseEstimator = new SwerveDrivePoseEstimator(
            kDriveKinematics,
            Rotation2d.fromDegrees(getGyroRotation()),
            getModulePositions(),
            initialPose);

        // Register alliance-change notification
        allianceSelection.addAllianceChangeNotification(this::resetPoseForAlliance);
    }

    @Override
    public void periodic() {
        // Update and log gyro inputs via AdvantageKit
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("DriveTrain/Gyro", gyroInputs);

        // Update all modules (each calls io.updateInputs + Logger.processInputs internally)
        for (SwerveModule module : swerveModules) {
            module.periodic();
        }

        // Update odometry + vision
        updateOdometry();

        // AdvantageKit output logging
        Logger.recordOutput("DriveTrain/Pose", getPose());
        Logger.recordOutput("DriveTrain/MeasuredStates", getMeasuredStates());
        Logger.recordOutput("DriveTrain/SetpointStates", getSetpointStates());
        Logger.recordOutput("DriveTrain/GyroAngle", getGyroRotation());
        Logger.recordOutput("DriveTrain/GyroPitch", getGyroPitch());
        Logger.recordOutput("DriveTrain/GyroVelocity", getAngularVelocity());
        Logger.recordOutput("DriveTrain/SpeedAvg", speedAvg);

        ChassisSpeeds robotSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = getRobotSpeeds();
        Logger.recordOutput("DriveTrain/RobotRelXVelocity", robotSpeeds.vxMetersPerSecond);
        Logger.recordOutput("DriveTrain/RobotRelYVelocity", robotSpeeds.vyMetersPerSecond);
        Logger.recordOutput("DriveTrain/FieldRelXVelocity", fieldSpeeds.vxMetersPerSecond);
        Logger.recordOutput("DriveTrain/FieldRelYVelocity", fieldSpeeds.vyMetersPerSecond);

        // Elastic / SmartDashboard updates
        if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kSlow)) {
            for (SwerveModule module : swerveModules) {
            module.updateElastic();
            }
            SmartDashboard.putNumber("Drive Bus Volt", swerveModules[0].getDriveBusVoltage());
        }

        if (DataLogUtil.isMyLogRotation(logRotationKey, RotationWhenDisabled.kNormal)) {
            checkForFaultConditions();

            SmartDashboard.putNumber("Drive Speed", speedAvg);
            SmartDashboard.putNumber("Drive X Velocity", fieldSpeeds.vxMetersPerSecond);
            SmartDashboard.putNumber("Drive Y Velocity", fieldSpeeds.vyMetersPerSecond);
            SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
            SmartDashboard.putBoolean("Drive isBrakeMode", !isDriveModeCoast());
            SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
            SmartDashboard.putNumber("Drive Gyro Rotation", getGyroRotation());
            SmartDashboard.putNumber("Drive AngVel", getAngularVelocity());
            SmartDashboard.putNumber("Drive Pitch", getGyroPitch());
            SmartDashboard.putBoolean("Vision Updating Odometry", useVisionForOdometry);

            Pose2d pose = getPose();
            SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
            SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
            SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());
        }
    }

    @Override
    public void simulationPeriodic() {
        // figure out chassis rotation rate based on simulated module states
        ChassisSpeeds simSpeeds = kDriveKinematics.toChassisSpeeds(getMeasuredStates());

        // Feed rotation rate into the simulated gyro so yaw changes realistically
        if (gyroIO instanceof GyroIOSim gyroSim) {
            gyroSim.updateSimState(Math.toDegrees(simSpeeds.omegaRadiansPerSecond), SwerveConstants.dt);
        }
    }


    // Gyro methods

    /**
     * Verifies if the gyro is still connected and reading.
     * @return true = gyro is connected, false = not connected
     */
    public boolean isGyroReading() {
        return gyroInputs.connected;
    }

    /**
     * Gets the raw gyro yaw (can be greater than 360 degrees).
     * @return raw gyro yaw, in degrees (positive = left, negative = right)
     */
    private double getGyroRaw() {
        return gyroInputs.yawDeg;
    }

    /**
     * Gets the raw gyro pitch.
     * @return raw gyro pitch, in degrees (positive = nose up, negative = nose down)
     */
    private double getGyroPitchRaw() {
        return gyroInputs.pitchDeg;
    }

    /**
     * Zeros the gyro pitch in software to the current pitch.
     */
    public void resetGyroPitch() {
        pitchZero = getGyroPitchRaw();
    }

    /**
     * Zeros the gyro yaw and pitch in software to the current readings.
     */
    public void zeroGyro() {
        yawZero  = getGyroRaw();
        pitchZero = getGyroPitchRaw();
    }

    /**
     * Zeros the gyro yaw in software against a specified heading.
     * @param currentHeading current robot heading compared to the desired zero angle, in degrees
     */
    public void zeroGyroRotation(double currentHeading) {
        yawZero = getGyroRaw() - currentHeading;
    }

    /**
     * Returns the calibrated gyro yaw. Most code should use getPoseAngle() instead,
     * which also incorporates AprilTag corrections from the pose estimator.
     * @return gyro angle, in degrees, -180 to +180 (positive = left, negative = right)
     */
    public double getGyroRotation() {
        return MathBCR.normalizeAngle(getGyroRaw() - yawZero);
    }

    /**
     * Gets the gyro pitch, zeroed to the calibrated pitch offset.
     * @return gyro pitch, in degrees (positive = nose up, negative = nose down)
     */
    public double getGyroPitch() {
        return getGyroPitchRaw() - pitchZero;
    }

    /**
     * Gets the angular velocity of the gyro.
     * @return gyro angular velocity, in degrees per second (positive = turning left)
     */
    public double getAngularVelocity() {
        return gyroInputs.yawVelocityDegPerSec;
    }


    // Swerve drive methods

    /**
     * Configures the motors and encoders for every swerve module.
     * Normally called automatically in constructors. Use this to force re-configuration after a brownout.
     * NOTE: includes blocking calls that will delay robot code.
     */
    public void configureSwerveModules() {
        for (SwerveModule module : swerveModules) {
            module.configSwerveModule();
        }
    }

    /**
     * Sets the drive train to coast or brake mode.
     * NOTE: may delay robot code ~250ms if the mode changes.
     * @param setCoast true = coast mode, false = brake mode
     */
    public void setDriveModeCoast(boolean setCoast) {
        for (SwerveModule module : swerveModules) {
            module.setMotorModeCoast(setCoast);
        }
        SmartDashboard.putString("Drive Mode", setCoast ? "Coast" : "Brake");
    }

    /**
     * Gets whether the drive train is in coast mode.
     * @return true = coast mode, false = brake mode
     */
    public boolean isDriveModeCoast() {
        // assume all modules are in same mode
        return swerveModules[0].isMotorModeCoast();
    }

    /**
     * Sets the drive motors to FOC or trapezoidal commutation mode.
     * Takes effect for the next request sent to the motors.
     * @param setFOC true = FOC mode, false = trapezoidal mode
     */
    public void setDriveMotorsFOC(boolean setFOC) {
        for (SwerveModule module : swerveModules) {
            module.setDriveMotorFOC(setFOC);
        }
    }

    /**
     * Sets the percent output of all drive motors.
     * @param percentOutput percent output, -1 to +1
     */
    public void setDriveMotorsOutput(double percentOutput) {
        for (SwerveModule module : swerveModules) {
            module.setDriveMotorPercentOutput(percentOutput);
        }
    }

    /**
     * Sets the percent output of all turning motors.
     * @param percentOutput percent output, -1 to +1
     */
    public void setTurningMotorsOutput(double percentOutput) {
        for (SwerveModule module : swerveModules) {
            module.setTurnMotorPercentOutput(percentOutput);
        }
    }

    /**
     * Sets the facing of all wheels to a given angle.
     * NOTE: sets the absolute facing, does not optimize to nearest ±180 degrees.
     * @param angle desired wheel facing in degrees, -180 to +180 (positive = left, 0 = front)
     */
    public void setWheelFacings(double angle) {
        for (SwerveModule module : swerveModules) {
            module.setWheelFacing(angle);
        }
    }

    /**
     * Sets the facing of a single wheel.
     * @param wheel wheel index: 0 = FL, 1 = FR, 2 = BL, 3 = BR
     * @param angle desired wheel facing in degrees, -180 to +180
     */
    public void setWheelFacing(int wheel, double angle) {
        if (wheel >= 0 && wheel < swerveModules.length) {
            swerveModules[wheel].setWheelFacing(angle);
        }
    }

    /**
     * Stops all drive and turning motors.
     */
    public void stopMotors() {
        for (SwerveModule module : swerveModules) {
            module.stopMotors();
        }
    }

    /**
     * Sets the states of the swerve modules.
     * @param desiredStates desired states: [0] FL, [1] FR, [2] BL, [3] BR
     * @param isOpenLoop true = open-loop voltage, false = closed-loop velocity
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        // Re-discretize (same approach as DriveTrain to cancel rotation skew)
        ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates);
        chassisSpeeds = ChassisSpeeds.discretize(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
            SwerveConstants.dt);

        desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d());
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
    }

    /**
     * Drives the robot using desired velocity and orientation.
     * @param xSpeed  x speed in m/s (positive = toward red DS)
     * @param ySpeed  y speed in m/s (positive = left from blue DS)
     * @param rot     angular rate in rad/s (positive = turn left)
     * @param driveType kRobotRelative, kDriverRelative, or kFieldRelative
     * @param isOpenLoop true = open-loop, false = closed-loop velocity
     */
    public void drive(double xSpeed, double ySpeed, double rot, DriveType driveType, boolean isOpenLoop) {
        drive(xSpeed, ySpeed, rot, new Translation2d(), driveType, isOpenLoop);
    }

    /**
     * Drives the robot using desired velocity and orientation, with a custom center of rotation.
     * @param xSpeed                 x speed in m/s
     * @param ySpeed                 y speed in m/s
     * @param rot                    angular rate in rad/s
     * @param centerOfRotationMeters center of rotation in meters from robot center
     * @param driveType              kRobotRelative, kDriverRelative, or kFieldRelative
     * @param isOpenLoop             true = open-loop, false = closed-loop velocity
     */
    public void drive(double xSpeed, double ySpeed, double rot,
        Translation2d centerOfRotationMeters, DriveType driveType, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeed;

        switch (driveType) {
            default:
            case kRobotRelative:
            chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, rot);
            break;
            case kFieldRelative:
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()));
            break;
            case kDriverRelative:
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()).plus(driverAngleToStation));
            break;
        }

        SwerveModuleState[] swerveModuleStates =
            kDriveKinematics.toSwerveModuleStates(chassisSpeed, centerOfRotationMeters);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }


    // Driver angle methods

    /**
     * Gets the driver's angle relative to their driver station.
     * @return driver's angle, in degrees
     */
    public double getDriverAngle() {
        return driverAngleToStation.getDegrees();
    }

    /**
     * Resets the driver angle. Call this when the robot is facing away from the driver.
     */
    public void resetDriverAngle() {
        driverAngleToStation = Rotation2d.fromDegrees(
            -getGyroRotation() + (allianceSelection.getAlliance() == Alliance.Blue ? 0 : 180));
    }


    // Odometry methods

    /**
     * Gets the currently-estimated robot pose on the field.
     * @return robot Pose2d:
     *   X = meters from Blue DS field edge (positive = toward Red DS),
     *   Y = meters from right field edge from Blue DS (positive = left),
     *   angle = field heading (0 = facing away from Blue DS, positive = left)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the currently-estimated robot angle on the field.
     * @return robot angle in degrees (0 = facing away from Blue DS, positive = left)
     */
    public double getPoseAngle() {
        return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    /**
     * Resets the pose estimator to the specified pose.
     * Also resets the gyro, which is required for the pose to properly reset.
     * @param pose new robot pose
     */
    public void resetPose(Pose2d pose) {
        DataLogUtil.writeMessage("Drive.resetPose with set angle =", pose.getRotation().getDegrees(),
            ", gyro pre =", getGyroRotation(), ", pose pre =", pose.getRotation().getDegrees());
        zeroGyroRotation(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(getGyroRotation()), getModulePositions(), pose);
    }

    /**
     * Resets the pose when the alliance changes. Rotates robot orientation 180° if at origin.
     * @param alliance new alliance
     */
    public void resetPoseForAlliance(Alliance alliance) {
        Pose2d pose = getPose();
        if (pose.getY() < 0.2 && pose.getX() < 0.2) {
            double rotation = MathBCR.normalizeAngle(pose.getRotation().getDegrees() - 180);
            resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(rotation)));
            CommandScheduler.getInstance().schedule(driveStopCommand);
        }
    }

    /**
     * Returns the robot chassis speeds in the field frame of reference.
     * @return ChassisSpeeds in the field frame
     */
    public ChassisSpeeds getRobotSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getChassisSpeeds(), Rotation2d.fromDegrees(getGyroRotation()));
    }

    /**
     * Returns the robot chassis speeds in the robot frame of reference.
     * @return ChassisSpeeds in the robot frame
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kDriveKinematics.toChassisSpeeds(getMeasuredStates());
    }

    /**
     * Reads the current swerve module states (measured).
     * @return SwerveModuleState[]: [0] FL, [1] FR, [2] BL, [3] BR
     */
    public SwerveModuleState[] getModuleStates() {
        return getMeasuredStates();
    }

    /**
     * Reads the current swerve module positions (for odometry).
     * @return SwerveModulePosition[]: [0] FL, [1] FR, [2] BL, [3] BR
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * Logs path-follower drive outputs. AdvantageKit handles module logging automatically,
     * but this method is preserved for compatibility with ChoreoFollower and similar code.
     * @param timeNow current FPGA timestamp (unused; AdvantageKit timestamps automatically)
     */
    public void updateDriveLogModuleOutputs(long timeNow) {
        Logger.recordOutput("DrivePathFollower/curPose2d", getPose());
        Logger.recordOutput("DrivePathFollower/BusVoltage", swerveModules[0].getDriveBusVoltage());
        Logger.recordOutput("DrivePathFollower/FLDriveOutput", swerveModules[0].getDriveOutputPercent());
        Logger.recordOutput("DrivePathFollower/FRDriveOutput", swerveModules[1].getDriveOutputPercent());
        Logger.recordOutput("DrivePathFollower/BLDriveOutput", swerveModules[2].getDriveOutputPercent());
        Logger.recordOutput("DrivePathFollower/BRDriveOutput", swerveModules[3].getDriveOutputPercent());
    }


    // Axis lock and fine-control - effect applied in DriveWithJoysticksAdvanced.

    /** Lock x-axis movement. */
    public void lockX() { xLocked = true; }

    /** Lock y-axis movement. */
    public void lockY() { yLocked = true; }
    
    /** Lock rotation. */
    public void lockTurn() { turnLocked = true; }


    /** @return true = x movement locked */
    public boolean isXLocked()    { return xLocked; }
    
    /** @return true = y movement locked */
    public boolean isYLocked()    { return yLocked; }
    
    /** @return true = rotation locked */
    public boolean isTurnLocked() { return turnLocked; }

    /** Unlock all joystick axes. */
    public void unlockAll() {
        xLocked    = false;
        yLocked    = false;
        turnLocked = false;
    }

    /**
     * Set fine control. Enabled = robot moves slower; disabled = normal speed.
     * @param enabled true = fine control, false = normal
     */
    public void setFineControl(boolean enabled) { fineControl = enabled; }

    /** @return true = fine control enabled */
    public boolean isFineControlEnabled() { return fineControl; }


    // Fault checking and CANbus status

    /**
     * Checks if the CANbus and gyro are working.
     * @return true = something is not working, false = all ok
     */
    public boolean canBusError() {
        return swerveModules[0].getDriveBusVoltage() < 7.0
            || swerveModules[0].getDriveTemp() < 5.0
            || !isGyroReading();
    }

    /**
     * Sets whether vision is used in odometry.
     * @param enabled true = use vision, false = don't use vision
     */
    public void setVisionForOdometryState(boolean enabled) {
        useVisionForOdometry = enabled;
    }

    /**
     * Enables or disables fast logging (every cycle vs. every 10 cycles).
     * @param enabled true = log every cycle, false = log every 10 cycles
     */
    @Override
    public void enableFastLogging(boolean enabled) {
        fastLogging = enabled;
    }


    // Private helpers

    /**
     * Updates the pose estimator using wheel odometry and vision cameras.
     * Logic is identical to DriveTrain.updateOdometry().
     */
    private void updateOdometry() {
        poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions());

        for (PhotonCameraWrapper camera : cameras) {
            PhotonPipelineResult camResult = camera.getLatestResult();
            if (camResult != null) {
                Optional<EstimatedRobotPose> result =
                    camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(), camResult);
                if (result.isPresent()) {
                    EstimatedRobotPose camPose = result.get();
                    SmartDashboard.putNumber("Vision " + camera.getName() + " X",   camPose.estimatedPose.toPose2d().getX());
                    SmartDashboard.putNumber("Vision " + camera.getName() + " Y",   camPose.estimatedPose.toPose2d().getY());
                    SmartDashboard.putNumber("Vision " + camera.getName() + " rot", camPose.estimatedPose.toPose2d().getRotation().getDegrees());

                    if (useVisionForOdometry && camResult.hasTargets()
                        && camResult.getBestTarget().getPoseAmbiguity() <= 0.3) {
                        PhotonTrackedTarget bestTarget = camResult.getBestTarget();
                        SmartDashboard.putNumber("Vision " + camera.getName() + " AprilTag", bestTarget.getFiducialId());
                        poseEstimator.addVisionMeasurement(
                            camPose.estimatedPose.toPose2d(),
                            camPose.timestampSeconds,
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

        ChassisSpeeds robotSpeeds = getRobotSpeeds();
        speedAvg = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    /** Checks for fault conditions and records sticky faults if any exist. */
    private void checkForFaultConditions() {
        recordedStickyFault = recordedStickyFault && RobotPreferences.isStickyFaultActive();

        if (!recordedStickyFault) {
            if (!isGyroReading()) {
                RobotPreferences.recordStickyFaults("DriveTrain-Gyro");
                DataLogUtil.writeMessageEcho("STICKY FAULT DriveTrain: Gyro not reading");
                recordedStickyFault = true;
            }
            for (SwerveModule module : swerveModules) {
                recordedStickyFault = recordedStickyFault || module.checkForFaultConditions();
            }
        }
    }

    /** Returns all measured module states (for logging and getChassisSpeeds). */
    private SwerveModuleState[] getMeasuredStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /** Returns all setpoint module states (for logging). */
    private SwerveModuleState[] getSetpointStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getSetpoint();
        }
        return states;
    }
}