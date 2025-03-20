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
import frc.robot.CoralAndElevatorState;

public class Constants {
    public static class CameraConstants
    {
        public static final Transform3d kRobotToRightCam = new Transform3d(new Translation3d(Inches.of(7), Inches.of(-12), Inches.of(7.750)), new Rotation3d(0, Math.toRadians(15), 0)); 
        public static final Transform3d kRobotToLeftCam = new Transform3d(new Translation3d(Inches.of(7), Inches.of(12), Inches.of(7.750)), new Rotation3d(0, Math.toRadians(15), 0)); 
        public static final Transform3d kRobotToUpCam = new Transform3d(new Translation3d(Inches.of(3.682), Inches.of(0), Inches.of(-37.079)), new Rotation3d(0, Math.toRadians(15), 0)); 

        public static final Translation2d kBlueReefCenter = new Translation2d(inchesToMeters(176.745), inchesToMeters(158.5));
        public static final Translation2d kRedReefCenter = new Translation2d(inchesToMeters(514.13), inchesToMeters(158.5));

        public static final int[] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        public static final int[] kStationIDs = {12, 13, 1, 2};

        public static final double kDistanceFromApriltagWhenScoring = inchesToMeters(23);
        public static final double kDistanceFromCoralToAprilTag = inchesToMeters(6.5);

        public static final double kDistanceFromStationTorRobot = inchesToMeters(22);
        public static final double kRobotToCoralIntakeLeftOffset = inchesToMeters(0.5); // center of the intake is 5 inches to the left of the center of the rboto

    }

    public static class ElevatorConstants
    {
        //Elevator Levels
        public static final double ELEVATOR_HOME_INCHES = 0;    //0
        public static final double ELEVATOR_L1_INCHES = 2;      //1
        public static final double ELEVATOR_L2_INCHES = 8.25;      //2
        public static final double ELEVATOR_L3_INCHES = 14.25;   //3
        public static final double ELEVATOR_L4_INCHES = 27.5;      //4
        public static final double ELEVATOR_MAX_INCHES = 28.7;    //Max


        public static final CoralAndElevatorState STOW_UP = new CoralAndElevatorState(0, 0, 0);
        public static final CoralAndElevatorState STOW_DOWN = new CoralAndElevatorState(0, .26, .26);
        public static final CoralAndElevatorState L1 = new CoralAndElevatorState(4, .26, .26);
        public static final CoralAndElevatorState L2 = new CoralAndElevatorState(9, .26, .26);
        public static final CoralAndElevatorState L3 = new CoralAndElevatorState(15.75, .26, .26);
        public static final CoralAndElevatorState L4 = new CoralAndElevatorState(27, .23, .23);
        public static final CoralAndElevatorState L4END = new CoralAndElevatorState(27.8, .23, .23);
        //public static final CoralAndElevatorState INTAKE = new CoralAndElevatorState(1.5, .04, .04, 1);
        public static final CoralAndElevatorState INTAKE = new CoralAndElevatorState(0, .04, .04, 1);
        public static final CoralAndElevatorState L2ALGAE = new CoralAndElevatorState(9, .26, .23, -1);
        public static final CoralAndElevatorState L3ALGAE = new CoralAndElevatorState(15.75, .26, .23, -1);
        

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
        public static final double ALGAE_DOWN_POINT = .12;
        public static final double ALGAE_STORE_POINT = .08;
    }
}
