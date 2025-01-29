package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public static class CameraConstants
    {
        public static final double kFieldLenght = 10;
        public static final double kFieldWidth = 10;
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 
        public static final   List<AprilTag> kApriltags = Arrays.asList(
            new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(0, 0,0))),
            new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(0, 0,0))),
            new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(0, 0,0))));
    }

}
