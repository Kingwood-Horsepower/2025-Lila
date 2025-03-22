package frc.robot.StateMachine;

import java.lang.Thread.State;

import frc.robot.StateMachine.states.TestingState;

public class StateMachine {

    private PlayerState currentState;
    private PlayerState testingState;
    private PlayerState startingState;

    public StateMachine(PlayerState startingState, PlayerState testingState)
    {
        this.startingState = startingState;
        this.testingState = testingState;
        currentState = startingState;
    }

    public void ChangeState(PlayerState newState)
    {
        // Don't change state if we are already changing state or we are changing to the same state
        if(currentState.isExiting || newState == currentState) {
            System.out.println("Error, invalid state (from player state machine)");
            return;
        }
        
            
        currentState.Exit();
        currentState = newState;
        currentState.Enter();
    }

    public PlayerState getState()
    {
        return currentState;
    }

    public void startStateMachine()
    {
        currentState = startingState;
        currentState.Enter();
    }

    public void startStateMachineTest()
    {
        currentState = testingState;
        currentState.Enter();
    }

}
