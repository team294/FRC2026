package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.StringUtil;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraWrapper extends SubsystemBase {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimatorCamera;
  
  private String cameraName;

  // Create Data Log Entries
  private final DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> dLogEstPose2D;
  private final BooleanLogEntry dLogConnected;
  private final BooleanLogEntry dLogTagPresent;
  private boolean priorConnected, priorTagPresent;


  /**
   * Creates a BCR wrapper for one PhotonVision camera.  Multiple PhotonCameraWrappers can be 
   * created, one for each camera on the robot.
   * @param robotToCam Transform3d specifying the location of the camera relative to the robot center
   * @param cameraName Name of this camera
   */
  public PhotonCameraWrapper(Transform3d robotToCam, String cameraName, AprilTagFieldLayout aprilTagFieldLayout) {
    this.cameraName = cameraName;

    dLogEstPose2D = StructLogEntry.create(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/estPose2d"), Pose2d.struct);
    dLogConnected = new BooleanLogEntry(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/Connected") );
    dLogTagPresent = new BooleanLogEntry(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/TagPresent") );

    DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Starting");

    // Create camera
    photonCamera = new PhotonCamera(cameraName);

    // Create pose estimator
    photonPoseEstimatorCamera = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCam);

    // Prime the DataLog to reduce delay when first enabling the robot
    long timeNow = RobotController.getFPGATime();
    dLogEstPose2D.append(Pose2d.kZero, timeNow);

    priorConnected = photonCamera.isConnected();
    dLogConnected.append(priorConnected, timeNow);
    SmartDashboard.putBoolean(StringUtil.buildString("PhotonVision ", cameraName, " Connected"), priorConnected);
    if (!priorConnected) {
      RobotPreferences.recordStickyFaults(cameraName);
      DataLogUtil.writeMessageEcho(StringUtil.buildString("STICKY FAULT ", cameraName));
    }
    priorTagPresent = false;
    dLogTagPresent.append(priorTagPresent, timeNow);

    DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Done");
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return cameraName;
  }

  /**
   * Gets a PhotonPipelineResult with the best target in Camera Pipeline result and the Camera enum. 
   * If there are no new targets, this method will return null. The best target is determined by the 
   * target sort mode in the PhotonVision UI.
   * @return the best target of the pipeline result
   */
  PhotonPipelineResult getLatestResult() {
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    return results.isEmpty() ? null : results.get(results.size()-1);
  }

  /**
   * Gets the estimated global pose of the robot on the field.
   * @param prevEstimatedRobotPose the current best guess at robot pose
   * @param latestResult the latest result from the photon vision pipeline
   * @return Optional of the fused camera observations to a single Pose2d on the field, and the time of
   * the observation. Assumes a planar field and the robot is always firmly on the ground.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult latestResult) {
    long timeNow = RobotController.getFPGATime();

    boolean isConnected = photonCamera.isConnected();
    if (isConnected != priorConnected) {
      // only write to datalog if this value has changed (saves space in the log)
      dLogConnected.append(isConnected, timeNow);
      SmartDashboard.putBoolean(StringUtil.buildString("PhotonVision ", cameraName, " Connected"), isConnected);
      priorConnected = isConnected;
      if (!isConnected) {
        RobotPreferences.recordStickyFaults(cameraName);
        DataLogUtil.writeMessageEcho(StringUtil.buildString("STICKY FAULT ", cameraName));
      }
    }

    // New for 2026:
    Optional<EstimatedRobotPose> newPoseOptional = photonPoseEstimatorCamera.estimateCoprocMultiTagPose(latestResult);
    if (newPoseOptional.isEmpty()) {
      newPoseOptional = photonPoseEstimatorCamera.estimateLowestAmbiguityPose(latestResult);
      // From 2025 code:
      //newPoseOptional = photonPoseEstimatorCamera.estimateClosestToReferencePose(latestResult, new Pose3d(prevEstimatedRobotPose));
    }

    boolean tagPresent = newPoseOptional.isPresent();
    if (tagPresent != priorTagPresent) {
      // only write to datalog if this value has changed (saves space in the log)
      dLogTagPresent.append(tagPresent, timeNow);
      priorTagPresent = tagPresent;
    }
    if (tagPresent) {
      EstimatedRobotPose newPose = newPoseOptional.get();
      dLogEstPose2D.append(newPose.estimatedPose.toPose2d(), timeNow);
    }
    return newPoseOptional;
  }

  /**
   * Calculates a pose estimator standard deviation matrix based on the number of AprilTags seen and 
   * the average distance to the AprilTag(s). Increases the standard deviation when the robot is farther
   * from the AprilTag (because the camera is less accurate).
   * <p> This algorithm is based upon the example code in PhotonVision.
   * @param estimatedPose Estimated robot pose from PhotonVision's pose estimator
   * @param targets List of targets seen by camera
   * @return pose estimator standard deviation matrix
   */
  public Matrix<N3, N1> getVisionMatrix(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    Matrix<N3, N1> curStdDevs;

    if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
        // Pose present. Start running Heuristic
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = photonPoseEstimatorCamera.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4)
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist * VisionConstants.kTagDistanceConst));
            curStdDevs = estStdDevs;
        }
    }

    return curStdDevs;
  }

  public void periodic() {
  }
}