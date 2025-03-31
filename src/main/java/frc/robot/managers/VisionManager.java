package frc.robot.managers;


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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.CameraConstants.*;

public class VisionManager {
    
    private static CameraSubsystem camera = new CameraSubsystem();  

    private static SwerveDriveManager swerveDriveManager;

    private final Field2d m_field = new Field2d();

    public VisionManager(SwerveDriveManager swerveDriveManager){
        this.swerveDriveManager = swerveDriveManager;
        

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putString("CameraLeftOdometry", "0");
        SmartDashboard.putNumber("CameraLeftOdometry(rotation)", Math.toDegrees(0));
        SmartDashboard.putString("CameraRightOdometry", "0");
        SmartDashboard.putNumber("CameraRightOdometry(rotation)", Math.toDegrees(0));
        SmartDashboard.putString("Robot Translation", "0");
        SmartDashboard.putNumber("Robot rotation", Math.toDegrees(0));
        SmartDashboard.putNumber("Robot Gyro", Math.toDegrees(0));
    }

    public void printScoringPosition(){
        for (int id : kRedReefIDs) {
            System.out.println(id + ", right: " + camera.getCoralScoreTransform(id, true).toString());
            System.out.println(id + ", left: " + camera.getCoralScoreTransform(id, false).toString());
        }
        for (int id : kBlueReefIDs) {
            System.out.println(id + ", right: " + camera.getCoralScoreTransform(id, true).toString());
            System.out.println(id + ", left: " + camera.getCoralScoreTransform(id, false).toString());
        }
    }


    public void setLowTrustInCameraRotation()
    {
        Matrix<N3, N1> matrix = MatBuilder.fill( Nat.N3(), Nat.N1(),1, 1, 9999);
        swerveDriveManager.setVisionTrust(matrix);
    }

    public void setHighTrustInCameraRotation()
    {
        Matrix<N3, N1> matrix = MatBuilder.fill( Nat.N3(), Nat.N1(),1, 1, 5 );
        swerveDriveManager.setVisionTrust(matrix);
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
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        //System.out.println("swerveRot: " + drivetrain.getRobotPose().getRotation().getRadians());
        //System.out.println("swervePos: " + drivetrain.getRobotPose().getTranslation());

        if (camera != null) {
            //Right camera
            var visionEst = camera.getEstimatedGlobalRightPose();
            visionEst.ifPresent(
                    est -> {

                        //Ignore results that contain anything other than reef,s april tag
                        boolean hasBadAprilTag = false;

                        for(PhotonTrackedTarget aprilTag : est.targetsUsed){
                            boolean isGood = false;
                            if(isBlue)
                            {
                                for(int id : kBlueReefIDs){
                                    isGood = isGood || aprilTag.fiducialId == id;
                                }
                            
                            }else
                            {
                                for(int id : kRedReefIDs){
                                    isGood = isGood || aprilTag.fiducialId == id;
                                }
                            }
                            
                            hasBadAprilTag = hasBadAprilTag || !isGood;
                       }
                       if(!hasBadAprilTag)
                       {
                        //swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        SmartDashboard.putString("CameraRightOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraRightOdometry(rotation)", est.estimatedPose.getRotation().getAngle());;
                       }
                        
                        
                        
                    });
            //Left camera

            visionEst = camera.getEstimatedGlobalLeftPose();
            visionEst.ifPresent(
                    est -> {
                        //Ignore results that contain anything other than reef,s april tag
                        boolean hasBadAprilTag = false;

                        for(PhotonTrackedTarget aprilTag : est.targetsUsed){
                            boolean isGood = false;
                            if(isBlue)
                            {
                                for(int id : kBlueReefIDs){
                                    isGood = isGood || aprilTag.fiducialId == id;
                                }
                            
                            }else
                            {
                                for(int id : kRedReefIDs){
                                    isGood = isGood || aprilTag.fiducialId == id;
                                }
                            }
                            
                            hasBadAprilTag = hasBadAprilTag || !isGood;
                       }
                       if(!hasBadAprilTag)
                       {
                        //swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
                        SmartDashboard.putString("CameraLeftOdometry", est.estimatedPose.getTranslation().toString());
                        SmartDashboard.putNumber("CameraLeftOdometry(rotation)",  est.estimatedPose.getRotation().getAngle());
                       }
                        
                    });

            // visionEst = camera.getEstimatedGlobalUpPose();
            // visionEst.ifPresent(
            //         est -> {
            //             //Ignore results that contain anything other than reef,s april tag
            //             boolean hasBadAprilTag = false;
            // visionEst = camera.getEstimatedGlobalUpPose();
            // visionEst.ifPresent(
            //         est -> {
            //             //Ignore results that contain anything other than reef,s april tag
            //             boolean hasBadAprilTag = false;

            //             for(PhotonTrackedTarget aprilTag : est.targetsUsed){
            //                 boolean isGood = false;
            //                 if(isBlue)
            //                 {
            //                     for(int id : kBlueStationIDs){
            //                         isGood = isGood || aprilTag.fiducialId == id;
            //                     }
                            
            //                 }else
            //                 {
            //                     for(int id : kRedStationIDs){
            //                         isGood = isGood || aprilTag.fiducialId == id;
            //                     }
            //                 }
                            
            //                 hasBadAprilTag = hasBadAprilTag || !isGood;
            //            }
            //            if(!hasBadAprilTag)
            //            {
            //             swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
            //             SmartDashboard.putString("CameraUpOdometry", est.estimatedPose.getTranslation().toString());
            //             SmartDashboard.putNumber("CameraUpOdometry(rotation)",  est.estimatedPose.getRotation().getAngle());
                
                        //swerveDriveManager.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds));
           //            }
            //        });          
        }

        SmartDashboard.putString("Robot Translation", swerveDriveManager.getRobotPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot rotation", swerveDriveManager.getRobotPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Robot Gyro", Math.toDegrees(swerveDriveManager.drivetrain.getPigeon2().getYaw().getValueAsDouble()));

        m_field.setRobotPose(swerveDriveManager.getRobotPose());
    }

    private void addVisionMeasurement(Pose2d pose, double timestamp)
    {
        Pose2d newPose = new Pose2d(pose.getTranslation(), swerveDriveManager.getRobotPose().getRotation());
        swerveDriveManager.addVisionMeasurement(newPose, timestamp);
    }

    public Pose2d getRobotScoringPosition(boolean isRightCoral){
        // var target = getBestDownTargetOptional();
        // if(target.isPresent()){

        //     int targetId = target.get().fiducialId;
        //     for (int reefId : kReefIDs) {
        //         if (targetId == reefId) {
        //             return camera.getCoralScoreTransform(targetId, isRightCoral);
        //         }           
        //     }
        // }
        // if we don't have a target, estimate the closest april tag based on the robot's current position

        List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        if(isBlue)
        {
            
            for(int id : kBlueReefIDs){
                reefPoses.add(camera.getCoralScoreTransform(id, isRightCoral));
            }
        
        }else
        {
            for(int id : kRedReefIDs){
                reefPoses.add(camera.getCoralScoreTransform(id, isRightCoral));
            }
        }
        
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    public Pose2d getRobotScoringPosition(int scoringTag, boolean isRightCoral) {
        return camera.getCoralScoreTransform(scoringTag, isRightCoral);
    }

    /**
     * Returns the closest scoring position of the robot, then which one is l4.
     * 
     */    
    public Pose2d getClosestRobotScoringPosition()
    {
        Pose2d rightPose = getRobotScoringPosition(true);
        Pose2d leftPose = getRobotScoringPosition(false);

        return swerveDriveManager.getRobotPose().nearest(List.of(rightPose, leftPose));

    }

    public Pose2d getRobotIntakePosition() {
        // var target = getBestDownTargetOptional();
        // if(target.isPresent()){
        //     int targetId = target.get().fiducialId;
        //     for (int reefId : kStationIDs) {
        //         if (targetId == reefId) {
        //             return camera.getStationPose2d(targetId);
        //         }           
        //     }
        // }
        // if we don't have a target, estimate the closest april tag based on the robot's current position
              List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        if(isBlue)
        {
            
            for(int id : kBlueStationIDs){
                reefPoses.add(camera.getStationPose2d(id));
            }
        
        }else
        {
            for(int id : kRedStationIDs){
                reefPoses.add(camera.getStationPose2d(id));
            }
        }
        
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    public Pose2d getRobotIntakePosition(int scoringTag) {
        return camera.getStationPose2d(scoringTag);
    }

    public Pose2d getRobotDealgeafyPosition(){
        // var target = getBestDownTargetOptional();
        // if(target.isPresent()){

        //     int targetId = target.get().fiducialId;
        //     for (int reefId : kReefIDs) {
        //         if (targetId == reefId) {
        //             return camera.getDealgeafyPose2d(targetId);
        //         }           
        //     }
        // }
        // if we don't have a target, estimate the closest april tag based on the robot's current position


        List<Pose2d> reefPoses  = new ArrayList<Pose2d>();    
        boolean isBlue = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
        if(isBlue)
        {
            for(int id : kBlueReefIDs){
                reefPoses.add(camera.getDealgeafyPose2d(id));
            }
        
        }else
        {
            for(int id : kRedReefIDs){
                reefPoses.add(camera.getDealgeafyPose2d(id));
            }
        }
        return swerveDriveManager.getRobotPose().nearest(reefPoses);

    }

    public CameraSubsystem getCameraSubsystem() {
        return camera;
    }

}
