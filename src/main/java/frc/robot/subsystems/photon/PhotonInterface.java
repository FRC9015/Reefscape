package frc.robot.subsystems.photon;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private PhotonCamera bow, port;
  AprilTagFieldLayout fieldLayout;

  Transform3d bowPose =
      new Transform3d(
          new Translation3d(
              -Units.Meters.convertFrom(20, Inch),
              -Units.Meters.convertFrom(1, Inch),
              Units.Meters.convertFrom(5, Inch)),
          new Rotation3d(
              0, -Units.Radians.convertFrom(15, Degree), Units.Radians.convertFrom(180, Degree)));

  Transform3d portPose =
      new Transform3d(
          new Translation3d(
              -Units.Meters.convertFrom(3, Inch),
              -Units.Meters.convertFrom(16, Inch),
              Units.Meters.convertFrom(7, Inch)),
          new Rotation3d(
              0, -Units.Radians.convertFrom(15, Degree), Units.Radians.convertFrom(270, Degree)));

  PhotonPoseEstimator photonPoseEstimatorBow;
  PhotonPoseEstimator photonPoseEstimatorPort;

  public PhotonInterface() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    bow = new PhotonCamera("Bow");
    port = new PhotonCamera("Starboard");

    photonPoseEstimatorBow =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, bowPose);
    photonPoseEstimatorBow.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonPoseEstimatorPort =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, portPose);
    photonPoseEstimatorPort.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("April Tag", bow.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/TwoTag", bow.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/Number", bow.getLatestResult().getTargets().size());
  }

  public Translation2d get2DEstimatedPose() {
    if (bow.getLatestResult().getTargets().size() == 0) {
      return new Translation2d(0, 0);
    }
    System.out.println(bow.getLatestResult().getTargets().size());

    return photonPoseEstimatorBow
        .update(bow.getLatestResult())
        .get()
        .estimatedPose
        .toPose2d()
        .getTranslation();
  }

  // makes an estimated pose out of the date from the starboard camera
  public Optional<EstimatedRobotPose> getEstimatedBowPose() {
    if (bow.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }

    return photonPoseEstimatorBow.update(bow.getLatestResult());
  }

  public Optional<EstimatedRobotPose> getEstimatedPortPose() {
    if (port.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }

    return photonPoseEstimatorPort.update(port.getLatestResult());
  }
}
