package frc.robot.StateMachine.states;

import frc.robot.StateMachine.PlayerState;

public class IntakeState extends PlayerState{
    public IntakeState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.swerveDriveManager.setSwerveToSlowDriveCommand().schedule();
        //Elevator to 1 (Intake)
        //Move Rollers
        System.out.println("Entered Intake State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onTriggerUnpressed(){
        //Elevator to 0, stop rollers
        player.stateMachine.ChangeState(player.driveState);
    }

}
