package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AlignToL4Constants;
import frc.robot.managers.SwerveDriveManager;
import frc.robot.managers.VisionManager;

public class AlignToReefCommand extends AlignCommand {

    private BooleanSupplier isRight;

    /**
     * Aligns to the nearest coral scoring position. It will not align to l4 if
     * @param swerveDriveManager
     * @param visionManager
     * @param isRight
     * @param isL4
     */
    public AlignToReefCommand(SwerveDriveManager swerveDriveManager, VisionManager visionManager, BooleanSupplier isRight) {
        super(swerveDriveManager, visionManager);
        this.isRight = isRight;
    }

    @Override
    public Pose3d initializeGoal() {
        //if (isL4.getAsBoolean()) System.out.println("align initialize initialized at l4");
        Pose2d newGoal = null;
        //if(goal == null) {
            if(isRight == null) newGoal = visionManager.getClosestRobotScoringPosition();
            else newGoal = visionManager.getRobotScoringPosition(isRight.getAsBoolean());
        //}
        return new Pose3d(newGoal);
    }
    

}
