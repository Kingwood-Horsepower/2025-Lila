package frc.robot.StateMachine.states;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;

import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.managers.SwerveDriveManager;

public class AlignmentState extends PlayerState{
    private boolean isRight = true;

    private BooleanSupplier isL4 = () -> player.coralAndElevatorSubsystem.getScoringLevel() == 4;

    private Command alignToRightReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->true, isL4);
    private Command alignToLeftReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->false, isL4);

    private Command alignToClosestReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, null, isL4);

    private Command swerveTestCommand = new DriveToPoseCommand(player.swerveDriveManager, player.visionManager);



    public AlignmentState(){
        super();
        //When the alignment is not working it gives control back to the player, only x robot centric
        player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.getSwerveDriveScoringCommand());
    }

    public void setIsRight(boolean isRight) {
        this.isRight = isRight;
    } 

    @Override public void Enter()
    {
        super.Enter();
        alignToClosestReefCommand.schedule();
        //swerveTestCommand.schedule();
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.getSwerveDriveScoringCommand());
        System.out.println("Entered Alignment State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onBumperPressed(){
        player.stateMachine.ChangeState(player.scoringState);
    }

    @Override public void onY(){
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        alignToClosestReefCommand.schedule();
    }
    @Override public void onA(){
        player.coralAndElevatorSubsystem.decrementElevatorScoringLevel();
        alignToClosestReefCommand.schedule();
    }

    @Override public void onBack()
    {
        player.coralAndElevatorSubsystem.moveDown();
        super.onBack();
    }

    
    @Override public void onPovLeft(){
        alignToLeftReefCommand.schedule();
    }
    @Override public void onPovRight(){
        alignToRightReefCommand.schedule();
    }


}
