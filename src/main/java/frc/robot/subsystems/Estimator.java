package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;

public class Estimator {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public Estimator(String cameraName, Transform3d robotToCam) {
        // Create a new PhotonCamera with the given name
        camera = new PhotonCamera(cameraName);

        // Create a new PhotonPoseEstimator
        estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }
}
