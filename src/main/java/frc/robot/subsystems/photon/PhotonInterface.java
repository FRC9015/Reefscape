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
  private PhotonCamera starboard, port;
  AprilTagFieldLayout fieldLayout;

  Transform3d starboardPose =
      new Transform3d(
          new Translation3d(
              Units.Meters.convertFrom(15, Inch),
              -Units.Meters.convertFrom(15, Inch),
              Units.Meters.convertFrom(5, Inch)),
          new Rotation3d(
              0,
              Units.Radians.convertFrom(-15, Degree),
              Units.Radians.convertFrom(90, Degree)));

  Transform3d portPose =
      new Transform3d(
          new Translation3d(
              Units.Meters.convertFrom(15, Inch),
              -Units.Meters.convertFrom(15, Inch),
              Units.Meters.convertFrom(5, Inch)),
          new Rotation3d(
              0,
              Units.Radians.convertFrom(-15, Degree),
              Units.Radians.convertFrom(90, Degree)));

  PhotonPoseEstimator photonPoseEstimatorStarboard;
  PhotonPoseEstimator photonPoseEstimatorPort;

  public PhotonInterface() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    starboard = new PhotonCamera("Starboard");
    port = new PhotonCamera("Port");

    photonPoseEstimatorStarboard =
        new PhotonPoseEstimator(fieldLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        starboardPose);
    photonPoseEstimatorStarboard.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonPoseEstimatorPort =
        new PhotonPoseEstimator(fieldLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        portPose);
    photonPoseEstimatorPort.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("April Tag", starboard.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/TwoTag", starboard.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/Number", starboard.getLatestResult().getTargets().size());
  }

  public Translation2d get2DEstimatedPose() {
    if (starboard.getLatestResult().getTargets().size() == 0) {
      return new Translation2d(0, 0);
    }
    System.out.println(starboard.getLatestResult().getTargets().size());

    return photonPoseEstimatorStarboard
        .update(starboard.getLatestResult())
        .get()
        .estimatedPose
        .toPose2d()
        .getTranslation();
  }

  public Optional<EstimatedRobotPose> getEstimatedStarboardPose() {
    if (starboard.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }
    // else if ((get2DEstimatedPose().getX() > 16.4846 || get2DEstimatedPose().getX() < 0)) {
    //   return Optional.empty();
    // } else if ((get2DEstimatedPose().getY() > 8.1026 || get2DEstimatedPose().getY() < 0)) {
    //   return Optional.empty();
    // }

    return photonPoseEstimatorStarboard.update(starboard.getLatestResult());
  }
  
  public Optional<EstimatedRobotPose> getEstimatedPortPose() {
    if (port.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }
    // else if ((get2DEstimatedPose().getX() > 16.4846 || get2DEstimatedPose().getX() < 0)) {
    //   return Optional.empty();
    // } else if ((get2DEstimatedPose().getY() > 8.1026 || get2DEstimatedPose().getY() < 0)) {
    //   return Optional.empty();
    // }

    return photonPoseEstimatorPort.update(port.getLatestResult());
  }
}
