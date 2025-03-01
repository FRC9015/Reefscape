package frc.robot.subsystems.photon;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonInterface extends SubsystemBase {
  private PhotonCamera bow, starboard;
  AprilTagFieldLayout fieldLayout;

  Transform3d bowPose =
      new Transform3d(
          new Translation3d(
              Units.Meters.convertFrom(15, Inch),
              Units.Meters.convertFrom(0, Inch),
              Units.Meters.convertFrom(7, Inch)),
          new Rotation3d(
              0, Units.Radians.convertFrom(15, Degree), Units.Radians.convertFrom(0, Degree)));

  Transform3d starboardPose =
      new Transform3d(
          new Translation3d(
              Units.Meters.convertFrom(0, Inch),
              -Units.Meters.convertFrom(15, Inch),
              Units.Meters.convertFrom(7, Inch)),
          new Rotation3d(
              0, Units.Radians.convertFrom(15, Degree), Units.Radians.convertFrom(270, Degree)));

  PhotonPoseEstimator photonPoseEstimatorBow;
  PhotonPoseEstimator photonPoseEstimatorStarboard;

  public PhotonInterface() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    bow = new PhotonCamera("Bow");
    starboard = new PhotonCamera("Starboard");

    photonPoseEstimatorBow =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, bowPose);
    photonPoseEstimatorBow.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonPoseEstimatorStarboard =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, starboardPose);
    photonPoseEstimatorStarboard.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean(
        "Tag Starboard", starboard.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/TwoTag", starboard.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/Number", starboard.getLatestResult().getTargets().size());
  }

  // makes an estimated pose out of the date from the starboard camera
  public Optional<EstimatedRobotPose> getEstimatedBowPose() {
    if (bow.getAllUnreadResults().size() == 0) {

      return Optional.empty();
    }

    return photonPoseEstimatorBow.update(bow.getLatestResult());
  }

  public Optional<EstimatedRobotPose> getEstimatedStarboardPose(Pose2d robotPose) {
    if (starboard.getAllUnreadResults().size() == 0) {
      return Optional.empty();
    }
    photonPoseEstimatorStarboard.setReferencePose(robotPose);
    return photonPoseEstimatorStarboard.update(starboard.getLatestResult());
  }

  public Optional<EstimatedRobotPose> bestPose(Pose2d robotPose) {

    if (starboard.getAllUnreadResults().size() == 0 || bow.getAllUnreadResults().size() == 0) {
      System.out.println(1);
      return Optional.empty();
    } else if (starboard.getAllUnreadResults().size() > 0
        || bow.getAllUnreadResults().size() == 0) {
      System.out.println(2);
      return getEstimatedStarboardPose(robotPose);
    } else if (starboard.getAllUnreadResults().size() == 0
        || bow.getAllUnreadResults().size() > 0) {
      System.out.println(3);
      return getEstimatedBowPose();
    }

    if (starboard.getAllUnreadResults().size() > 0 || bow.getAllUnreadResults().size() > 0) {
      if (starboard.getLatestResult().getBestTarget().getPoseAmbiguity()
          < bow.getLatestResult().getBestTarget().getPoseAmbiguity()) {
        System.out.println(4);
        return getEstimatedStarboardPose(robotPose);
      } else {
        System.out.println(5);
        return getEstimatedBowPose();
      }
    }
    System.out.println(6);
    return Optional.empty();
  }
}
