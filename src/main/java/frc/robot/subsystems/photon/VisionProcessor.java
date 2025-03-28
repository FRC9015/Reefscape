package frc.robot.subsystems.photon;

import java.util.function.Consumer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionProcessor {

  // For reference, in photon web it is called "reefscapeV5-640-480yolov8n.rknn"

  // Create PhotonCamera instances using the same names as set in PhotonVision UI.
  private final PhotonCamera topCamera = new PhotonCamera("Top");

  // Class IDs for coral, algae, L2, L3, and L4
  private final int algaeClassID = 0;
  private final int coralClassID = 1;
  private final int l2ClassID = 2;
  private final int l3ClassID = 3;
  private final int l4ClassID = 4;

  // Consumer to notify when coral or algae is detected
  private Consumer<TargetType> targetDetectedConsumer;

  public VisionProcessor(Consumer<TargetType> targetDetectedConsumer) {
    this.targetDetectedConsumer = targetDetectedConsumer;
  }

  public void processVision() {
    TargetType targetType = checkCameraForTarget(topCamera);
    targetDetectedConsumer.accept(targetType);
  }

  private TargetType checkCameraForTarget(PhotonCamera camera) {
    // Get the latest result from PhotonVision pipeline.
    PhotonPipelineResult result = camera.getLatestResult();

    // Check if any targets are detected
    if (!result.hasTargets()) {
      return TargetType.NONE;
    }

    // Iterate through all detected targets.
    for (PhotonTrackedTarget target : result.getTargets()) {
      int classID = target.getDetectedObjectClassID();
      if (classID == coralClassID) {
        System.out.println("Desired target detected: coral");
        return TargetType.CORAL;
      } else if (classID == algaeClassID) {
        System.out.println("Desired target detected: algae");
        return TargetType.ALGAE;
      } else if (classID == l2ClassID) {
        System.out.println("Desired target detected: L2");
        return TargetType.L2;
      } else if (classID == l3ClassID) {
        System.out.println("Desired target detected: L3");
        return TargetType.L3;
      } else if (classID == l4ClassID) {
        System.out.println("Desired target detected: L4");
        return TargetType.L4;
      }
    }

    return TargetType.NONE;
  }

  public TargetType isCoralDetected() {
    return checkCameraForTarget(topCamera);
  }

  public enum TargetType {
    NONE,
    ALGAE,
    CORAL,
    L2,
    L3,
    L4
  }
}
