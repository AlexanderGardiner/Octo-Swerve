/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
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

 package frc.robot.Subsystems.vision;

 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.util.Units;
 
 public class Constants {
     static class DriveTrainConstants {
         static final double kMaxSpeed = 3.0; // meters per second
         static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
         static final double kTrackWidth = 0.381 * 2; // meters
         static final double kWheelRadius = 0.0508; // meters
         static final int kEncoderResolution = 4096;
         static final double distancePerPulse = 2 * Math.PI * kWheelRadius / (double) kEncoderResolution;
     }
 
     public static class FieldConstants {
         public static final double length = Units.feetToMeters(54);
         public static final double width = Units.feetToMeters(27);
     }
 
     public static class LeftCameraVisionConstants {
        public static final Transform3d robotToCam =
                 new Transform3d(
                         new Translation3d(0.0508, -0.1143, 1.0414),
                         new Rotation3d(
                                 0, -0.349,
                                 -0.2617)); // Cam mounted facing forward, half a meter forward of center, half a meter up
         // from center.
         public static final String cameraName = "Left_Camera";
     }

     public static class VisionConstants {
        public static final Transform3d robotToCam =
                 new Transform3d(
                         new Translation3d(0.0,0.1302, 0.0),
                         new Rotation3d(
                                 0, -0.174533,
                                 0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
         // from center.
         public static final String cameraName = "Right_Camera";
     }
 }