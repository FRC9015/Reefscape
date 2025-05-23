package frc.robot.subsystems.photon;

import java.util.function.Consumer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionProcessor {

  // Create PhotonCamera instances using the same names as set in PhotonVision UI.
  private final PhotonCamera topCamera = new PhotonCamera("Top");

  // Class IDs for coral and algae
  private final int algaeClassID = 0;
  private final int coralClassID = 1;

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
    CORAL
  }
}
