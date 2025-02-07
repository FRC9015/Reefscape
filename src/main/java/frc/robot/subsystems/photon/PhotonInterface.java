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

  Transform3d camPose =
      new Transform3d(

          // new Translation3d(Units.inchesToMeters(12.25), -Units.inchesToMeters(10.875),
          // Units.inchesToMeters(11)),
          new Translation3d(
              Units.Meters.convertFrom(15, Inch),
              -Units.Meters.convertFrom(15, Inch),
              Units.Meters.convertFrom(5, Inch)), // 24.974 //11.6661	//10.634 //
          new Rotation3d(
              0,
              Units.Radians.convertFrom(-15, Degree),
              Units.Radians.convertFrom(90, Degree))); // 37.4// try negative pitch
  PhotonPoseEstimator photonPoseEstimator;

  public PhotonInterface() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Couldn't Find April Tag Layout File");
      e.printStackTrace();
    }

    starboard = new PhotonCamera("Starboard");
    // port = new PhotonCamera("Port");

    photonPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camPose);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

    return photonPoseEstimator
        .update(starboard.getLatestResult())
        .get()
        .estimatedPose
        .toPose2d()
        .getTranslation();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    if (starboard.getLatestResult().getTargets().size() == 0) {

      return Optional.empty();
    }
    // else if ((get2DEstimatedPose().getX() > 16.4846 || get2DEstimatedPose().getX() < 0)) {
    //   return Optional.empty();
    // } else if ((get2DEstimatedPose().getY() > 8.1026 || get2DEstimatedPose().getY() < 0)) {
    //   return Optional.empty();
    // }

    return photonPoseEstimator.update(starboard.getLatestResult());
  }
}
