package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToStationCommand extends AlignCommand {

    public AlignToStationCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager) {
        super(swerveDriveManager, visionManager);
    }

    @Override
    public Pose3d initializeGoal() {
        return new Pose3d(visionManager.getRobotIntakePosition());
    }
    
}
