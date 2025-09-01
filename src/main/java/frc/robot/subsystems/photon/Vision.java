package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.photon.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final VisionIO[] io;

  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Matrix<N3, N1> curStdDevs;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  //   public TargetObservation getLastTargetObersevation(int camera) {
  //     return inputs[camera].latestTargetObservation;
  //   }

  //   public int[] getTagIDs(int cameraIndex) {
  //     return inputs[cameraIndex].tagIds;
  //   }

  public Optional<PoseObservation> getNewestPoseObservation(int cameraIndex) {
    PoseObservation[] observations = inputs[cameraIndex].poseObservations;
    if (observations.length != 0) {
      return Optional.of(observations[0]);
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        Optional<Pose3d> tagPose = CameraConstants.aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > 0.1) // Cannot be high ambiguity

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > CameraConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > CameraConstants.aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = CameraConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var target : tagPoses) {
          numTags++;
          avgDist +=
              target
                  .toPose2d()
                  .getTranslation()
                  .getDistance(observation.pose().toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = CameraConstants.kSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = CameraConstants.kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 5)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
        }

        // Send vision observation
        consumer.accept(observation.pose().toPose2d(), observation.timestamp(), curStdDevs);
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));

      if (!robotPosesAccepted.isEmpty()) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/AcceptedRobotPoses",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      }
      if (!robotPosesRejected.isEmpty()) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RejectedRobotPoses",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      }

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);

      Logger.recordOutput("Vision/StdDevs", curStdDevs);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
