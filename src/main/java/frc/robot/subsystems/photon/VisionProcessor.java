package frc.robot.subsystems.photon;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionProcessor {

  // Create a PhotonCamera instance using the same name as set in PhotonVision UI.
  private final PhotonCamera camera = new PhotonCamera("ObjDetc");

  // This is the class ID you want to act on.
  private final int coralClassID = 0;

  public boolean ifCoralDetected() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        int classID = target.getDetectedObjectClassID();
        if (classID == coralClassID) {
          return true;
        }
      }
    }
    return false;
  }

  public void processVision() {
    // Get the latest result from PhotonVision pipeline.
    PhotonPipelineResult result = camera.getLatestResult();

    // Check if any targets are detected
    if (!result.hasTargets()) {
      System.out.println("No targets detected.");
      return;
    }

    boolean desiredTargetFound = false;

    // Iterate through all detected targets.
    for (PhotonTrackedTarget target : result.getTargets()) {
      int classID = target.getDetectedObjectClassID();
      if (classID == coralClassID) {
        desiredTargetFound = true;
        System.out.println("Desired target detected: coral");
        // add some sort of movement logic
      }
    }

    if (!desiredTargetFound) {
      System.out.println("Desired target not detected.");
    }
  }
}
