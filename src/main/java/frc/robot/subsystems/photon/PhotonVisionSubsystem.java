package frc.robot.subsystems.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d; // Not needed since we are using multiple cameras
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVisionSubsystem extends SubsystemBase {

  // Create two PhotonVision cameras to track AprilTags
  private final PhotonCamera camera1 = new PhotonCamera(CameraConstants.CAMERA_1_NAME);
  private final PhotonCamera camera2 = new PhotonCamera(CameraConstants.CAMERA_2_NAME);

  // Pose estimators for each camera
  private final PhotonPoseEstimator poseEstimator1;
  private final PhotonPoseEstimator poseEstimator2;

  // The robot's pose estimator that combines odometry and vision data
  private final SwerveDrivePoseEstimator swervePoseEstimator;

  // Field layout containing the AprilTag positions
  private AprilTagFieldLayout fieldLayout;

  // Constructor initializes the PhotonVision subsystem. It sets up AprilTag field layout and pose
  // estimators for the cameras. @param swervePoseEstimator The robot's pose estimator for
  // integrating vision data.
  public PhotonVisionSubsystem(SwerveDrivePoseEstimator swervePoseEstimator) {
    this.swervePoseEstimator = swervePoseEstimator;

    // Load the AprilTag field layout from the 2025 Reefscape game field
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    // Initialize pose estimators for each camera
    poseEstimator1 =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy
                .MULTI_TAG_PNP_ON_COPROCESSOR, // Strategy that selects the pose with the least
            // uncertainty for multi-tag detection
            CameraConstants.CAMERA_1_TO_ROBOT);

    poseEstimator2 =
        new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CameraConstants.CAMERA_2_TO_ROBOT);

    // Set fallback strategy if multi-tag detection fails
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
  }

  // Called periodically by the robot framework. This method updates the robot's position using
  // vision data, and enables possibility to add filtering.
  @Override
  public void periodic() {
    updateOdometry();
  }

  // Updates the robot's pose estimation by processing AprilTag detections from both cameras. If
  // both cameras detect tags, it averages their poses to improve accuracy. The pose estimation is
  // then added to the swerve pose estimator.
  // This logic is very experimental and will need adjusted with stronger logic.
  public void updateOdometry() {
    // Get 3D pose estimates from each camera
    Optional<EstimatedRobotPose> estimate1 = poseEstimator1.update(camera1.getLatestResult());
    Optional<EstimatedRobotPose> estimate2 = poseEstimator2.update(camera2.getLatestResult());

    if (estimate1.isPresent() && estimate2.isPresent()) {
      // Use the full 3D poses from both cameras
      Pose3d visionPose1 = estimate1.get().estimatedPose;
      Pose3d visionPose2 = estimate2.get().estimatedPose;

      // Average X, Y, and Z positions
      Pose3d combinedPose =
          new Pose3d(
              (visionPose1.getX() + visionPose2.getX()) / 2,
              (visionPose1.getY() + visionPose2.getY()) / 2,
              (visionPose1.getZ() + visionPose2.getZ()) / 2, // Preserve depth (Z)
              visionPose1.getRotation() // Use rotation from camera 1 (or interpolate)
              );

      double timestamp = Timer.getFPGATimestamp() - estimate1.get().timestampSeconds;
      swervePoseEstimator.addVisionMeasurement(
          combinedPose.toPose2d(), timestamp); // Convert back to Pose2d
    } else if (estimate1.isPresent()) {
      Pose3d visionPose = estimate1.get().estimatedPose;
      double timestamp = Timer.getFPGATimestamp() - estimate1.get().timestampSeconds;
      swervePoseEstimator.addVisionMeasurement(visionPose.toPose2d(), timestamp);
    } else if (estimate2.isPresent()) {
      Pose3d visionPose = estimate2.get().estimatedPose;
      double timestamp = Timer.getFPGATimestamp() - estimate2.get().timestampSeconds;
      swervePoseEstimator.addVisionMeasurement(visionPose.toPose2d(), timestamp);
    }
  }
}
