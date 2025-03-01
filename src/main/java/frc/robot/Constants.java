package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Constants {
    public static class CameraConstants
    {
        public static final Distance kFieldLenght = Meters.of(12.29);
        public static final Distance kFieldWidth = Meters.of(2.87);

        public static final Transform3d kRobotToRightCam = new Transform3d(new Translation3d(Inches.of(7), Inches.of(-12), Inches.of(7.750)), new Rotation3d(0, Math.toRadians(15), 0)); 
        public static final Transform3d kRobotToLeftCam = new Transform3d(new Translation3d(Inches.of(7), Inches.of(12), Inches.of(7.750)), new Rotation3d(0, Math.toRadians(15), 0));
        public static final Transform3d kRobotToUpCam = new Transform3d(new Translation3d(Inches.of(3.682), Inches.of(0), Inches.of(37.355)), new Rotation3d(0, Math.toRadians(15), 0)); 

        public static final Translation2d kReefCenter = new Translation2d(inchesToMeters(176.745), inchesToMeters(158.5));
        public static final double kDistanceFromApriltagWhenScoring = inchesToMeters(24);
        public static final double kDistanceFromCoralToAprilTag = inchesToMeters(6);
        public static final double kDistanceFromStationTorRobot = inchesToMeters(22);


        public static final   List<AprilTag> kApriltags = Arrays.asList(
            new AprilTag(21, new Pose3d(new Translation3d(Inches.of(169), Inches.of(113),  Inches.of(45)), new Rotation3d(0, 0, 3*Math.PI/2))),
            new AprilTag(4, new Pose3d(new Translation3d(Inches.of(376), Inches.of(0),  Inches.of(23.6)), new Rotation3d(0, 0,Math.PI/2))),
            new AprilTag(22, new Pose3d(new Translation3d(Inches.of(169), Inches.of(98),  Inches.of(6)), new Rotation3d(0, 0, Math.PI)))
            );

    }

    public static class ElevatorConstants
    {
        // ELEVATOR LEVELS
        public static final double ELEVATOR_HOME_INCHES = 0;    //0
        public static final double ELEVATOR_L1_INCHES = 3;      //1
        public static final double ELEVATOR_L2_INCHES = 9;      //2
        public static final double ELEVATOR_L3_INCHES = 15.5;   //3
        public static final double ELEVATOR_L4_INCHES = 26.5;      //4
        public static final double ELEVATOR_MAX_INCHES = 27.8;    //Max
    }

    public static class AutoConstants
    {
        //Can be 1, 2 or 3.  1 is the one closest to the center
        public static final StartCage startingPosition = StartCage.sigmaCage;

        //Can be 1 or 2. 1 is the one furthest from the starting position (to the right of the player)
        public static final TargetCoralStation targetStation = TargetCoralStation.leftStation;


        enum StartCage{
            edgeCage,
            sigmaCage, //Second cage
            goonCage //Center cage
        }
        enum TargetCoralStation{
            leftStation,
            rightStation
        }

        public static Translation2d getStartingPosition(){
            switch (startingPosition) {
                case edgeCage:
                    return new Translation2d(Units.inchesToMeters(318.428), Units.inchesToMeters(286.779));
                case sigmaCage:
                    return new Translation2d(Units.inchesToMeters(318.428), Units.inchesToMeters(242.855));
                case goonCage:             
                    return new Translation2d(Units.inchesToMeters(318.428), Units.inchesToMeters(199.947));
                default:
                    return null;
            }
        }
        

    }
    


    public static class AlgaeConstants
    {
        public static final double ALGAE_DOWN_POINT = .11;
        public static final double ALGAE_STORE_POINT = .08;
    }

    public static class CoralIntakeConstants
    {
        /*notes on geometry
        angle of lower bar limits
        71 deg up
        -60 deg down
        angle of CoM onshape
        12 deg up from lower bar
        88 deg up
        -44 deg down
        weight of arm
        10 lbs
        values calculated in recalc aaaaa
        */ 
        public static final double kG = 0.59; //V
        public static final double kV = 0.88; //Vs/rad
        // public static final double armStowPositionPerpendicular = 71/360;
        // public static final double armDownPositionPerpendicular = -44/360;
        // public static final double armScorePositionPerpendicular = armStowPositionPerpendicular - 0.26;
        
    }
}
