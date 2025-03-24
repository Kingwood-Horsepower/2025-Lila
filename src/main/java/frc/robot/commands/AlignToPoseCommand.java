package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.commands.AlignCommand;

public class AlignToPoseCommand extends AlignCommand {
    private Pose2d goal;

    public AlignToPoseCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, Pose2d goal) {
        super(swerveDriveManager, visionManager);
        this.goal = goal;
    }

    @Override
    public Pose3d initializeGoal() {
        return new Pose3d(goal);
    }
    
}
