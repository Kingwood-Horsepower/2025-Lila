package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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

        public static final double kDistanceFromApriltagWhenScoring = inchesToMeters(24.8 
        ); //Vertical distance from the center of the robot originally 23
        public static final double kDistanceFromCoralToAprilTag = inchesToMeters(6.5); //Orizzontal distance when scoring

        public static final double kDistanceFromApriltagWhenDealgeafy = inchesToMeters(20);

        //Decrease/Increase this to change intaking alignment in autonomous
        public static final double kDistanceFromStationTorRobot = inchesToMeters(17);


        public static final double kRightReefScoringOffset = inchesToMeters(1); // shifts the robot .5 inches toward the tag (to the left of the right reef)
        public static final double kLeftReefScoringOffset = inchesToMeters(0); // shifts the robot 1 inch away from the tag (to the left of the left reef)
    }

    // public static class AlignToL4Constants 
    // {
    //     public static final double ROBOT_TO_L4_DISTANCE = -inchesToMeters(6.3); //How much you have to go back when you score L4 (Compared to L3)
    // }

    public static class CoralIntakeConstants
    {
        public static final int GEARBOX_RATIO = 15; // just the ratio of maxplanetaries. not used
        public static final int SPROCKET_RATIO = 3;
        //The double set to the throughbore encoder after the end of zeroing
        public static final double CORAL_ZERO_POINT = .97;
    }

    public static class ElevatorConstants
    {
        public static final int ELEVATOR_GEAR_RATIO = 12;
        public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE = 1.281*Math.PI;
        public static final double ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS = ELEVATOR_GEAR_RATIO/ELEVATOR_SPROCKET_CIRCUMFERENCE;
    }

    public static class CoralAndElevatorConstants
    {

        // Coral and Elevator states 
        public static final CoralAndElevatorState STOW_UP = new CoralAndElevatorState(0, 0);
        public static final CoralAndElevatorState STOW_DOWN = new CoralAndElevatorState(0, .26);
        public static final CoralAndElevatorState L1 = new CoralAndElevatorState(4, .26);
        public static final CoralAndElevatorState L2 = new CoralAndElevatorState(10, .26); // original is 9 for elevator position
        public static final CoralAndElevatorState L3 = new CoralAndElevatorState(17.7, .26); // original is 17.2 for elevator position
        public static final CoralAndElevatorState L4 = new CoralAndElevatorState(28.7, .244);
        public static final CoralAndElevatorState INTAKE = new CoralAndElevatorState(0, .035, .5);// original speed was .8
        public static final CoralAndElevatorState L2ALGAE = new CoralAndElevatorState(4, .23, -1);
        public static final CoralAndElevatorState L3ALGAE = new CoralAndElevatorState(11.5, .23, -1);
        public static final double NORMAL_SCORING_SPEED = -0.25; 
    }

    
    public static class AlgaeConstants
    {
        public static final double ALGAE_DOWN_POINT = .12;
        public static final double ALGAE_STORE_POINT = .08;
        public static final double ALGAE_SCORING_POINT = .03;
        public static final double ALGAE_INTAKE_SPEED = -1.0;
        public static final double ALGAE_OUTTAKE_SPEED = 1.0;
        public static final double ALGAE_INWARD_INTAKED_PULL = -.1; // at the end of intaking, when i have an algae, move the rollers inward at a small speed to ensure the algae stays in
    }

    public static class AlignmentControllerConstants
    {
        //dont change controller tolerance unless you are advised to by someone who knows what it does, 
        //(i dont know what it does, i also do not use controller.atGoal() so I don't think it is used?)
        public static final double X_CONTROLLER_TOLERANCE = 0.5;
        public static final double Y_CONTROLLER_TOLERANCE = 0.5;
        public static final double THETA_CONTROLLER_TOLERANCE = 0.5;

        //you may need to tune these values if the robot continuously oscilates during alignment instead of ending.
        public static final double X_Kp = 7;
        public static final double Y_Kp = 7;
        public static final double THETA_Kp = 15;
        public static final double X_Kd = .5;
        public static final double Y_Kd = .5;
        public static final double THETA_Kd = .5;

        //you could also lower the ending tolerance of alignment to combat the robot continuously oscilating during alignment instead of ending
        // but this does lower the tolerance of alignment
        public static final double XY_ALIGNMENT_TOLERANCE = 0.03; // idk what these units are lmao i think they are in meters
        public static final double TEHTA_ALIGNMENT_TOLERANCE = 0.5; // degrees
    }
}
