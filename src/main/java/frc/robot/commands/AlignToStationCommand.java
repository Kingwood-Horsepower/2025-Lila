package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToStationCommand extends DriveToPoseCommand {

    public AlignToStationCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager) {
        super(swerveDriveManager, visionManager);
    }
    
    @Override
    public void initialize() {
        // reset controllers

        xController.reset(swerveDriveManager.getRobotPose().getX());
        yController.reset(swerveDriveManager.getRobotPose().getY());
        thetaController.reset(swerveDriveManager.getRobotPose().getRotation().getRadians());

        goal = new Pose3d(visionManager.getRobotIntakePosition());
    }
  

}
