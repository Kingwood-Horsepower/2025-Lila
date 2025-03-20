package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.managers.SwerveDriveManager;

public class AlignmentState extends PlayerState{
    private Command alignToReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->driverController.b().getAsBoolean());
    private Command alignToClosestReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, null);

    private Command swerveTestCommand = new DriveToPoseCommand(player.swerveDriveManager, player.visionManager);

    public AlignmentState(){
        super();
        //When the alignment is not working it gives control back to the player, only x robot centric
        player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.getSwerveDriveScoringCommand());
    }
    @Override public void Enter()
    {
        super.Enter();
        //alignToClosestReefCommand.schedule();
        swerveTestCommand.schedule();
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        System.out.println("Entered Alignment State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onBumperPressed(){
        player.stateMachine.ChangeState(player.scoringState);
    }

    @Override public void onX(){
        alignToReefCommand.schedule();
    }
    @Override public void onB(){
        alignToReefCommand.schedule();
    }

    @Override public void onY(){
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
    }
    @Override public void onA(){
        player.coralAndElevatorSubsystem.decrementElevatorScoringLevel();
    }

    @Override public void onBack()
    {
        player.coralAndElevatorSubsystem.moveDown();
        super.onBack();
    }


}
