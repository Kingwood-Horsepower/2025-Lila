package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class CameraConstants
    {
        public static final Distance kFieldLenght = Meters.of(12.29);
        public static final Distance kFieldWidth = Meters.of(2.87);
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(Inches.of(-8), Inches.of(-2.5), Inches.of(9.5)), new Rotation3d(0, -0.122 * Math.PI, Math.PI)); 
        public static final   List<AprilTag> kApriltags = Arrays.asList(
            new AprilTag(21, new Pose3d(new Translation3d(Inches.of(169), Inches.of(113),  Inches.of(45)), new Rotation3d(0, 0, 3*Math.PI/2))),
            new AprilTag(4, new Pose3d(new Translation3d(Inches.of(376), Inches.of(0),  Inches.of(23.6)), new Rotation3d(0, 0,Math.PI/2))),
            new AprilTag(22, new Pose3d(new Translation3d(Inches.of(169), Inches.of(98),  Inches.of(6)), new Rotation3d(0, 0, Math.PI)))
            );
    }

}
