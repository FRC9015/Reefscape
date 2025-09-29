// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d(), 0, 0, 0); // default values...
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(
      Rotation2d tx,
      Rotation2d ty,

      // width
      double targetHorizontalExtentPixels,
      // height
      double targetVerticalExtentPixels,
      double timestamp) {
    public TargetObservation(Rotation2d tx, Rotation2d ty, double timestamp) {
      this(tx, ty, 0, 0, timestamp);
    }

    public double getTargetHorizontalExtentPixels() {
      return targetHorizontalExtentPixels;
    }

    public double getTargetVerticalExtentPixels() {
      return targetVerticalExtentPixels;
    }
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      List<Short> tagId) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
