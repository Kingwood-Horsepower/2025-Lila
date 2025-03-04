package frc.robot.managers;

import static frc.robot.Constants.AutoConstants.getStartingPosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionManager {
    
    private static CameraSubsystem camera = new CameraSubsystem(); 

    private static CommandSwerveDrivetrain drivetrain;

    public VisionManager(CommandSwerveDrivetrain _drivetrain){
        drivetrain = _drivetrain;
    }

    /**
     * Returns the best target between the right and left camera
     * use the .ifPresent() method to check if you got a target
     */
    public Optional<PhotonTrackedTarget> getBestDownTargetOptional(){
        Optional<PhotonTrackedTarget> target = Optional.empty();
        if(camera.hasDownTarget())
        {
            target = Optional.of(camera.getBestDownTarget());
        }
        return target;
    }

    /**
     * Returns the best target of the up camera
     * use the .ifPresent() method to check if you got a target
     */
    public Optional<PhotonTrackedTarget> getBestUpTargetOptional(){
        Optional<PhotonTrackedTarget> target = Optional.empty();
        if(camera.hasUpTarget())
        {
            target = Optional.of(camera.getBestUpTarget());
        }
        return target;
    }

    public void UpdateRobotPosition() {
        //System.out.println("swerveRot: " + drivetrain.getRobotPose().getRotation().getRadians());
        //System.out.println("swervePos: " + drivetrain.getRobotPose().getTranslation());

        if (camera != null) {
            //Right camera
            var visionEst = camera.getEstimatedGlobalRightPose();
            visionEst.ifPresent(
                    est -> {
                        //System.out.println("right: " +est.estimatedPose.getTranslation());
                        SmartDashboard.putString("CameraRightOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraRightOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));

                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
            //Left camera

            visionEst = camera.getEstimatedGlobalLeftPose();
            visionEst.ifPresent(
                    est -> {
                        //System.out.println("left: " + est.estimatedPose.getTranslation());
                        SmartDashboard.putString("CameraLeftOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));
        
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    });
        }

        SmartDashboard.putString("Robot Translation", drivetrain.getRobotPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot rotation", Math.toDegrees(drivetrain.getRobotPose().getRotation().getDegrees()));
    }

    public Optional<Pose2d> getRobotScoringPosition(boolean isRightCoral){
        Optional<Pose2d> pose = Optional.empty();
        var target = getBestDownTargetOptional();
        if(target.isPresent()){
            pose = Optional.of(camera.getCoralScoreTransform(target.get().fiducialId, isRightCoral));
        }
        return pose;

    }
    public Optional<Pose2d> getRobotIntakePosition(){
        Optional<Pose2d> pose = Optional.empty();
        var target = getBestUpTargetOptional();
        if(target.isPresent()){
            pose = Optional.of(camera.getStationPose2d(target.get().fiducialId));
        }
        return pose;

    }

}
