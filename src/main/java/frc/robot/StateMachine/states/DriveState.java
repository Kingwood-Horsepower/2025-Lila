package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class DriveState extends PlayerState{
    public DriveState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        System.out.println("Entered Drive State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onTrigger(){
        player.stateMachine.ChangeState(player.intakeState);
    }
    @Override public void onY(){
        player.stateMachine.ChangeState(player.alignmentState);
    }
    @Override public void onA(){
        player.stateMachine.ChangeState(player.alignmentState);
    }

}
