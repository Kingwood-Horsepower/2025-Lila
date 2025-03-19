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
        //Elevator to 0
        //Stop Rollers
        System.out.println("Entered Drive State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onTriggerPressed(){
        player.stateMachine.ChangeState(player.intakeState);
    }
    @Override public void onY(){
        player.stateMachine.ChangeState(player.alignmentState);
    }

}
