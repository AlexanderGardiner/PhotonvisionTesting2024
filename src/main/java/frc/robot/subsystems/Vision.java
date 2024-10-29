/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

    Estimator stinkyCam;
    Estimator blinkyCam;
    Estimator pinkyCam;
    Estimator clydeCam;
    Estimator inkyCam;

    private Matrix<N3, N1> curStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision() {
        stinkyCam = new Estimator(kStinkyCameraName, ROBOT_TO_STINKY);
        blinkyCam = new Estimator(kBlinkyCameraName, ROBOT_TO_BLINKY);
        pinkyCam = new Estimator(kPinkyCameraName, ROBOT_TO_PINKY);
        clydeCam = new Estimator(kClydeCameraName, ROBOT_TO_CLYDE);
        inkyCam = new Estimator(kInkyCameraName, ROBOT_TO_INKY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");

            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);

            // Create simulated camera properties. These can be set to mimic your actual camera.
            var stinkyProp = new SimCameraProperties();
            stinkyProp.setCalibration(1280, 720,
                    MatBuilder.fill(Nat.N3(), Nat.N3(),
                            new double[] { 725.4584312171277, 0, 641.9084855962565, 0, 720.2065818357393,
                                    372.27555032904695, 0, 0, 1 }),
                    MatBuilder.fill(Nat.N8(), Nat.N1(), new double[] { 0.07571501830333022, -0.10251810538472172,
                            -0.0028631574695070884, -0.00303678096558035, 0.050458491273620475 }));

            // cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            // cameraProp.setCalibError(0.35, 0.10);
            // cameraProp.setFPS(15);
            // cameraProp.setAvgLatencyMs(50);
            // cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.

            // cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            // visionSim.addCamera(cameraSim, kRobotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    //     Optional<EstimatedRobotPose> visionEst = Optional.empty();
    //     for (var change : camera.getAllUnreadResults()) {
    //         visionEst = photonEstimator.update(change);
    //         updateEstimationStdDevs(visionEst, change.getTargets());

    //         if (Robot.isSimulation()) {
    //             visionEst.ifPresentOrElse(
    //                     est -> getSimDebugField()
    //                             .getObject("VisionEstimation")
    //                             .setPose(est.estimatedPose.toPose2d()),
    //                     () -> {
    //                         getSimDebugField().getObject("VisionEstimation").setPoses();
    //                     });
    //         }
    //     }
    //     return visionEst;
    // }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }
}