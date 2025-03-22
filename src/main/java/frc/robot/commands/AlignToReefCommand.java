package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AlignToL4Constants;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToReefCommand extends DriveToPoseCommand {

    private BooleanSupplier isRight;
    private BooleanSupplier isL4;

    public AlignToReefCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, BooleanSupplier isRight, BooleanSupplier isL4) {
        super(swerveDriveManager, visionManager);
        this.isRight = isRight;
        this.isL4 = isL4;
    }

    @Override
    public Pose3d initializeGoal() {
        Pose2d newGoal = null;
        //if(goal == null) {
            if(isRight == null) newGoal = visionManager.getClosestRobotScoringPosition();
            else newGoal = visionManager.getRobotScoringPosition(isRight.getAsBoolean());
        //}

        Transform2d l4Transform = new Transform2d(AlignToL4Constants.ROBOT_TO_L4_DISTANCE, 0, newGoal.getRotation());
        if (isL4.getAsBoolean()) newGoal.plus(l4Transform);

        return new Pose3d(newGoal);
    }
    

}
