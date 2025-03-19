package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;
import frc.robot.StateMachine.PlayerStateMachine;

public class DriveState extends PlayerState{
    public DriveState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.swerveDriveManager.setSwerveToNormalDriveCommand().schedule();
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
        else
            player.stateMachine.ChangeState(player.algeePickupState);
    }

}
