package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;

public class OffsetAlign {

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static final Pose2d REEF_OFFSET =
      new Pose2d(0, Units.inchesToMeters(6.5), Rotation2d.kZero);

  public static final Pose2d[] reefLocations = {
    aprilTagLayout.getTagPose(17).get().toPose2d(),
    aprilTagLayout.getTagPose(18).get().toPose2d(),
    aprilTagLayout.getTagPose(19).get().toPose2d(),
    aprilTagLayout.getTagPose(20).get().toPose2d(),
    aprilTagLayout.getTagPose(21).get().toPose2d(),
    aprilTagLayout.getTagPose(22).get().toPose2d()
  };

  public static Pose2d getPose2dFlipped(Pose2d pose) {
    return AllianceFlipUtil.apply(
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(Units.inchesToMeters(18.375), 0)
                        .rotateBy(pose.getRotation())),
            pose.getRotation().plus(Rotation2d.k180deg)));
  }

  public static Pose2d flipPose(Pose2d pose) {
    return AllianceFlipUtil.apply(pose);
  }

  public static Pose2d getPose2dReef(boolean isleft, Pose2d pose) {
    if (isleft) {
      return AllianceFlipUtil.apply(
          new Pose2d(
              pose.getTranslation()
                  .plus(
                      REEF_OFFSET
                          .getTranslation()
                          .rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                  .plus(
                      new Translation2d(Units.inchesToMeters(18.375), 0)
                          .rotateBy(pose.getRotation())),
              pose.getRotation().plus(Rotation2d.k180deg)));
    }

    return AllianceFlipUtil.apply(
        new Pose2d(
            pose.getTranslation()
                .minus(
                    REEF_OFFSET
                        .getTranslation()
                        .rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                .plus(
                    new Translation2d(Units.inchesToMeters(18.375), 0)
                        .rotateBy(pose.getRotation())),
            pose.getRotation().plus(Rotation2d.k180deg)));
  }
}
