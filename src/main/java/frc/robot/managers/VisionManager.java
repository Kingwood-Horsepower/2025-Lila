package frc.robot.managers;

import static frc.robot.Constants.AutoConstants.getStartingPosition;
import static frc.robot.Constants.CameraConstants.kReefIDs;

import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionManager {
    
    private static CameraSubsystem camera = new CameraSubsystem(); 

    private static CommandSwerveDrivetrain drivetrain;

    public VisionManager(CommandSwerveDrivetrain _drivetrain){
        drivetrain = _drivetrain;
        Matrix<N3, N1> matrix = MatBuilder.fill( Nat.N3(), Nat.N1(),1.0, 1.0, 99.0);
        drivetrain.setVisionMeasurementStdDevs(matrix);
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
                        
                       
                        
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                    });
            //Left camera

            visionEst = camera.getEstimatedGlobalLeftPose();
            visionEst.ifPresent(
                    est -> {
                        //System.out.println("left: " + est.estimatedPose.getTranslation());
                        SmartDashboard.putString("CameraLeftOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));
        
                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                    });
        }

        SmartDashboard.putString("Robot Translation", drivetrain.getRobotPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot rotation", Math.toDegrees(drivetrain.getRobotPose().getRotation().getDegrees()));
    }

    public Pose2d getRobotScoringPosition(boolean isRightCoral){
        var target = getBestDownTargetOptional();
        if(target.isPresent()){

            int targetId = target.get().fiducialId;
            for (int reefId : kReefIDs) {
                if (targetId == reefId) {
                    return camera.getCoralScoreTransform(targetId, isRightCoral);
                }           
            }
        }
        // if we don't have a target, estimate the closest april tag based on the robot's current position

        System.out.println("No target found, using estimate using robot position");

        List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        for (int id : kReefIDs) {
            reefPoses.add(camera.getCoralScoreTransform(id, isRightCoral));
        }
        return drivetrain.getRobotPose().nearest(reefPoses);

    }

    /**
     * Returns the closest scoring position of the robot.
     * Authomatically determines if the robot is closer to the right or left coral
     */    
    public Pose2d getClosestRobotScoringPosition()
    {
        Pose2d rightPose = getRobotScoringPosition(true);
        Pose2d leftPose = getRobotScoringPosition(false);

        return drivetrain.getRobotPose().nearest(List.of(rightPose, leftPose));

    }

    //Does not work currently
    private Optional<Pose2d> getRobotIntakePosition(){
        Optional<Pose2d> pose = Optional.empty();
        var target = getBestUpTargetOptional();
        if(target.isPresent()){
            pose = Optional.of(camera.getStationPose2d(target.get().fiducialId));
        }
        return pose;

    }

}
