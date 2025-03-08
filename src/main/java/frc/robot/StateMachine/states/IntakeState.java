package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class IntakeState extends PlayerState{
    public IntakeState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        System.out.println("Entered Intake State");
    }
    @Override public void Exit(){
        super.Exit();
    }

}
