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

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static class Vision {

        public static final String kStinkyCameraName = "Stinky";
        public static final String kBlinkyCameraName = "Blinky";
        public static final String kPinkyCameraName = "Pinky";
        public static final String kClydeCameraName = "Clyde";
        public static final String kInkyCameraName = "Inky";

        /*
         * X positive is forward from the front of the robot
         * Y positive is left from the front of the robot
         * Z positive is upwards
         * Positive roll is unknown because we didn't use it
         * Positive pitch is towards the front of the robot (CCW positive around the Y
         * axis)
         * Positive yaw is unknown because we didn't use it
         * Order of operations for rotations, first it rotates around the yaw, then it
         * does pitch
         */

        public static final Transform3d ROBOT_TO_INKY = new Transform3d(
                new Translation3d(0.5, 0.5, 0.5),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-14.5), Math.toRadians(15)));

        public static final Transform3d ROBOT_TO_BLINKY = new Transform3d(
                new Translation3d(0.5, -0.5, 0.5),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-13.5), Math.toRadians(345)));

        public static final Transform3d ROBOT_TO_PINKY = new Transform3d(
                new Translation3d(-0.5, 0.5, 0.5),
                new Rotation3d(0, Math.toRadians(-13), Math.toRadians(165)));

        public static final Transform3d ROBOT_TO_CLYDE = new Transform3d(
                new Translation3d(-0.5, -0.5, 0.5),
                new Rotation3d(0, Math.toRadians(-13), Math.toRadians(195)));

        public static final Transform3d ROBOT_TO_STINKY = new Transform3d(
                new Translation3d(-0.5, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-25), Math.toRadians(180)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2024Crescendo);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

}