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
        public static final Transform3d kRobotToUpCam = new Transform3d(new Translation3d(Inches.of(0), Inches.of(-3.5), Inches.of(40)), new Rotation3d(0, Math.toRadians(35), 0)); 

        public static final Translation2d kBlueReefCenter = new Translation2d(inchesToMeters(176.745), inchesToMeters(158.5));
        public static final Translation2d kRedReefCenter = new Translation2d(inchesToMeters(514.13), inchesToMeters(158.5));

        public static final int[] kBlueReefIDs = {17, 18, 19, 20, 21, 22}; 
        public static final int[] kRedReefIDs = {6, 7, 8, 9, 10, 11}; 

        public static final int[] kRedStationIDs = { 1, 2};
        public static final int[] kBlueStationIDs = {12, 13};

        public static final double kDistanceFromApriltagWhenScoring = inchesToMeters(23
        ); //Vertical distance from the center of the robot
        public static final double kDistanceFromCoralToAprilTag = inchesToMeters(6.5); //Orizzontal distance when scoring

        public static final double kDistanceFromApriltagWhenDealgeafy = inchesToMeters(20);

        public static final double kDistanceFromStationTorRobot = inchesToMeters(22);

        public static final double kRobotToCoralIntakeLeftOffset = inchesToMeters(0.5); // Center of the intake is 5 inches to the left of the center of the rboto
        public static final double kExtraLeftAlignmentAddition = inchesToMeters(1.25); 
    }

    public static class AlignToL4Constants 
    {
        public static final double ROBOT_TO_L4_DISTANCE = -inchesToMeters(7); //How much you have to go back when you score L4 (Compared to L3)
    }

    public static class ElevatorConstants
    {
        //Elevator states 
        public static final CoralAndElevatorState STOW_UP = new CoralAndElevatorState(0, 0, 0);
        public static final CoralAndElevatorState STOW_DOWN = new CoralAndElevatorState(0, .26, .26);
        public static final CoralAndElevatorState L1 = new CoralAndElevatorState(4, .26, .26);
        public static final CoralAndElevatorState L2 = new CoralAndElevatorState(8.5, .26, .26);
        public static final CoralAndElevatorState L3 = new CoralAndElevatorState(15.6, .26, .26);
        public static final CoralAndElevatorState L4 = new CoralAndElevatorState(28.7, .24, .24);
        public static final CoralAndElevatorState L4END = new CoralAndElevatorState(28.7, .24, .24);
        public static final CoralAndElevatorState INTAKE = new CoralAndElevatorState(0, .05, .05, .5);
        public static final CoralAndElevatorState L2ALGAE = new CoralAndElevatorState(4, .26, .23, -1);
        public static final CoralAndElevatorState L3ALGAE = new CoralAndElevatorState(10.75, .26, .23, -1);
    }

    
    public static class AlgaeConstants
    {
        public static final double ALGAE_DOWN_POINT = .12;
        public static final double ALGAE_STORE_POINT = .08;
    }
}
