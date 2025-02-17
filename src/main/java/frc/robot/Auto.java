package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {

    private final AutoFactory autoFactory;

    public Auto(CommandSwerveDrivetrain driveSubsystem)
    {   
        autoFactory = new AutoFactory(
            driveSubsystem::getRobotPose, // A function that returns the current robot pose
            driveSubsystem::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
            driveSubsystem // The drive subsystem
        );
    }



}
