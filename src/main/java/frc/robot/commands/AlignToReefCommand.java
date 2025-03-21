package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToReefCommand extends DriveToPoseCommand {

    private BooleanSupplier isRight;

    public AlignToReefCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, BooleanSupplier isRight) {
        super(swerveDriveManager, visionManager);
        this.isRight = isRight;
    }

    @Override
    public Pose3d initializeGoal() {
        if(goal == null) {
            if(isRight == null) goal = new Pose3d(visionManager.getClosestRobotScoringPosition());
            else goal = new Pose3d(visionManager.getRobotScoringPosition(isRight.getAsBoolean()));
        }
        return goal;
    }
    

}
