package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;
import frc.robot.commands.AlignCommand;

public class AlignDeAlgaeifyCommand extends AlignCommand {

    public AlignDeAlgaeifyCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager) {
        super(swerveDriveManager, visionManager);

        
    }

    @Override
    public Pose3d initializeGoal() {
        return new Pose3d(visionManager.getRobotDealgeafyPosition());
    }

    @Override
    public void end (boolean isInterrupted) {
        super.end(isInterrupted);
        
    }
    
}
