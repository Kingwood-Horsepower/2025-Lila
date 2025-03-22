package frc.robot.StateMachine.states;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.StateMachine.PlayerStateMachine;
import frc.robot.commands.AlignToReefCommand;

public class DriveState extends PlayerState{
    public DriveState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.swerveDriveManager.setSwerveToNormalDriveCommand().schedule();
        player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.setSwerveToNormalDriveCommand());
        System.out.println("Entered Drive State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onTriggerPressed(){
        player.stateMachine.ChangeState(player.intakeState);
    }
    @Override public void onY(){
        if(player.coralAndElevatorSubsystem.hasCoral())
            player.stateMachine.ChangeState(player.alignmentState);
            //return;
        else
            player.stateMachine.ChangeState(player.algeePickupState);
    }

    private BooleanSupplier isL4 = () -> player.coralAndElevatorSubsystem.getScoringLevel() == 4;

    private Command alignToRightReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->true, isL4);
    private Command alignToLeftReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->false, isL4);

    @Override public void onPovLeft(){
        if(player.coralAndElevatorSubsystem.hasCoral())
        {
            alignToLeftReefCommand.schedule();
        }
    }
    @Override public void onPovRight(){
        if(player.coralAndElevatorSubsystem.hasCoral())
        {
            alignToRightReefCommand.schedule();
        }
    }

}
