package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class Zone {
  private Translation2d point1;
  private Translation2d point2;
  private Translation2d originalPoint1;
  private Translation2d originalPoint2;
  private double fieldWidth = 16.21;
  private DriverStation.Alliance alliance;

  public Zone(
      Translation2d point1,
      Translation2d point2,
      Supplier<DriverStation.Alliance> allianceSupplier) {
    this.point1 = point1;
    this.point2 = point2;
    this.originalPoint1 = point1;
    this.originalPoint2 = point2;
    this.alliance = allianceSupplier.get();

    if (alliance.equals(DriverStation.Alliance.Red)) {
      this.point1 = new Translation2d(fieldWidth - point1.getX(), point1.getY());
      this.point2 = new Translation2d(fieldWidth - point2.getX(), point2.getY());
    }
  }

  public void updateToAlliance() {
    if (alliance.equals(DriverStation.Alliance.Red)) {
      this.point1 = new Translation2d(fieldWidth - originalPoint1.getX(), originalPoint1.getY());
      this.point2 = new Translation2d(fieldWidth - originalPoint2.getX(), originalPoint2.getY());
    } else {
      this.point1 = new Translation2d(originalPoint1.getX(), originalPoint1.getY());
      this.point2 = new Translation2d(originalPoint2.getX(), originalPoint2.getY());
    }
  }

  public boolean isIn(Pose2d pose) {
    boolean betweenX =
        pose.getX() > Math.min(point1.getX(), point2.getX())
            && pose.getX() < Math.max(point1.getX(), point2.getX());

    boolean betweenY =
        pose.getY() > Math.min(point1.getY(), point2.getY())
            && pose.getY() < Math.max(point1.getY(), point2.getY());

    return betweenX && betweenY;
  }
}
