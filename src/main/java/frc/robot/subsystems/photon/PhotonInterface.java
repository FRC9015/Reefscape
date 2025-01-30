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
  private PhotonCamera tagCam;
  AprilTagFieldLayout fieldLayout;

  Transform3d camPose =
      new Transform3d(

          // new Translation3d(Units.inchesToMeters(12.25), -Units.inchesToMeters(10.875),
          // Units.inchesToMeters(11)),
          new Translation3d(
              Units.Meters.convertFrom(1.911942, Inch),
              -Units.Meters.convertFrom(10.8125, Inch),
              Units.Meters.convertFrom(10.634, Inch)), // 24.974 //11.6661	//10.634 //
          new Rotation3d(
              0,
              Units.Radians.convertFrom(-30, Degree),
              Units.Radians.convertFrom(180, Degree))); // 37.4// try negative pitch
  PhotonPoseEstimator photonPoseEstimator;

  public PhotonInterface() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    tagCam = new PhotonCamera("tagCam0");

    photonPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camPose);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("April Tag", tagCam.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/TwoTag", tagCam.getLatestResult().getTargets().size() == 2);
    Logger.recordOutput("Tags/Number", tagCam.getLatestResult().getTargets().size());
  }

  public Translation2d get2DEstimatedPose() {
    if (tagCam.getLatestResult().getTargets().size() == 0) {
      return new Translation2d(0, 0);
    }
    System.out.println(tagCam.getLatestResult().getTargets().size());

    return photonPoseEstimator
        .update(tagCam.getLatestResult())
        .get()
        .estimatedPose
        .toPose2d()
        .getTranslation();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    if (tagCam.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }
    // else if ((get2DEstimatedPose().getX() > 16.4846 || get2DEstimatedPose().getX() < 0)) {
    //   return Optional.empty();
    // } else if ((get2DEstimatedPose().getY() > 8.1026 || get2DEstimatedPose().getY() < 0)) {
    //   return Optional.empty();
    // }

    return photonPoseEstimator.update(tagCam.getLatestResult());
  }
}
