package frc.robot.managers;

import static frc.robot.Constants.AutoConstants.getStartingPosition;
import static frc.robot.Constants.CameraConstants.*;

import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ejml.equation.IntegerSequence.For;
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

import static frc.robot.Constants.CameraConstants.*;

public class VisionManager {
    
    private static CameraSubsystem camera = new CameraSubsystem(); 

    private static SwerveDriveManager swerveDriveManager;

    public VisionManager(SwerveDriveManager swerveDriveManager){
        this.swerveDriveManager = swerveDriveManager;
        Matrix<N3, N1> matrix = MatBuilder.fill( Nat.N3(), Nat.N1(),1, 1, 99.0);
        swerveDriveManager.setVisionTrust(matrix);

        SmartDashboard.putString("CameraLeftOdometry", "0");
        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(0));
        SmartDashboard.putString("CameraRightOdometry", "0");
        SmartDashboard.putNumber("CameraRightOdometry(rotation)", Math.toDegrees(0));
        SmartDashboard.putString("Robot Translation", "0");
        SmartDashboard.putNumber("Robot rotation", Math.toDegrees(0));
    }

    public void printScoringPosition(){
        for (int id : kReefIDs) {
            System.out.println(id + ", right: " + camera.getCoralScoreTransform(id, true).toString());
            System.out.println(id + ", left: " + camera.getCoralScoreTransform(id, false).toString());
        }
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

                        
                        boolean hasBadAprilTag = false;

                        for(PhotonTrackedTarget i : est.targetsUsed){
                            boolean isGood = false;
                            for(int id : kReefIDs){
                                isGood = isGood || i.fiducialId == id;
                            }
                            hasBadAprilTag = hasBadAprilTag || !isGood;
                       }
                       if(!hasBadAprilTag)
                       {
                        swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        SmartDashboard.putString("CameraRightOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraRightOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));;
                       }
                        
                        
                        
                    });
            //Left camera

            visionEst = camera.getEstimatedGlobalLeftPose();
            visionEst.ifPresent(
                    est -> {
                        boolean hasBadAprilTag = false;

                        for(PhotonTrackedTarget i : est.targetsUsed){
                            boolean isGood = false;
                            for(int id : kReefIDs){
                                isGood = isGood || i.fiducialId == id;
                            }
                            hasBadAprilTag = hasBadAprilTag || !isGood;
                       }
                       if(!hasBadAprilTag)
                       {
                        swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        SmartDashboard.putString("CameraLeftOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));
                       }
                        
                    });

            // visionEst = camera.getEstimatedGlobalUpPose();
            // visionEst.ifPresent(
            //         est -> {
            //             SmartDashboard.putString("CameraUpOdometry", est.estimatedPose.getTranslation().toString());
            //             SmartDashboard.putNumber("CameraUpOdometry(rotation)", Math.toDegrees(est.estimatedPose.getRotation().getAngle()));
                
            //             swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
            //         });          
        }

        SmartDashboard.putString("Robot Translation", swerveDriveManager.getRobotPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot rotation", Math.toDegrees(swerveDriveManager.getRobotPose().getRotation().getDegrees()));
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
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    /**
     * Returns the closest scoring position of the robot.
     * Authomatically determines if the robot is closer to the right or left coral
     */    
    public Pose2d getClosestRobotScoringPosition()
    {
        Pose2d rightPose = getRobotScoringPosition(true);
        Pose2d leftPose = getRobotScoringPosition(false);

        return swerveDriveManager.getRobotPose().nearest(List.of(rightPose, leftPose));

    }

    public Pose2d getRobotIntakePosition(){
        var target = getBestDownTargetOptional();
        if(target.isPresent()){
            int targetId = target.get().fiducialId;
            for (int reefId : kStationIDs) {
                if (targetId == reefId) {
                    return camera.getStationPose2d(targetId);
                }           
            }
        }
        // if we don't have a target, estimate the closest april tag based on the robot's current position

        System.out.println("No target found, using estimate using robot position");

        List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        for (int id : kStationIDs) {
            reefPoses.add(camera.getStationPose2d(id));
        }
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    public Pose2d getRobotDealgeafyPosition(){
        var target = getBestDownTargetOptional();
        if(target.isPresent()){

            int targetId = target.get().fiducialId;
            for (int reefId : kReefIDs) {
                if (targetId == reefId) {
                    return camera.getDealgeafyPose2d(targetId);
                }           
            }
        }
        // if we don't have a target, estimate the closest april tag based on the robot's current position

        System.out.println("No target found, using estimate using robot position");

        List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        for (int id : kReefIDs) {
            reefPoses.add(camera.getDealgeafyPose2d(id));
        }
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    public CameraSubsystem getCameraSubsystem() {
        return camera;
    }

}
