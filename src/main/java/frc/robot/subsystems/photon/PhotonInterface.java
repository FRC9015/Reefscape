package frc.robot.subsystems.photon;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;

import java.io.IOException;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;


public class PhotonInterface extends SubsystemBase{
     private PhotonCamera tagCam;
    AprilTagFieldLayout fieldLayout;
    
    

    

	Transform3d camPose = new Transform3d(
			
        //new Translation3d(Units.inchesToMeters(12.25), -Units.inchesToMeters(10.875), Units.inchesToMeters(11)),
	    new Translation3d(Units.Meters.convertFrom(1.911942,Inch), -Units.Meters.convertFrom(10.8125,Inch), Units.Meters.convertFrom(10.634,Inch)),//24.974 //11.6661	//10.634 //	
        new Rotation3d(0, Units.Radians.convertFrom(-30,Degree), Units.Radians.convertFrom(180,Degree)));//37.4// try negative pitch
	PhotonPoseEstimator photonPoseEstimator;


    public PhotonInterface() {
        try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
		} catch (IOException e) {
			System.out.println("Couldn't Find April Tag Layout File");
			e.printStackTrace();
		}

		tagCam = new PhotonCamera("Tag_Camera");

		photonPoseEstimator =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,camPose);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        
    }

   
    @Override
    public void periodic() {
        
        SmartDashboard.putBoolean("April Tag", tagCam.getLatestResult().getTargets().size() == 2);
        Logger.recordOutput("Tags/TwoTag", tagCam.getLatestResult().getTargets().size() == 2);
        Logger.recordOutput("Tags/Number", tagCam.getLatestResult().getTargets().size());
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
		if (tagCam.getLatestResult().getTargets().size() == 0){
           
            return Optional.empty();
        }        

        return photonPoseEstimator.update(tagCam.getLatestResult());
	}

    
   
}
