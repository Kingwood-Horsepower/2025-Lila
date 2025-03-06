package frc.robot.StateMachine;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class PlayerState {
    
    public static CommandXboxController driverController;
    public static PlayerStateMachine player;

    public boolean isExiting = false;

    public PlayerState(){
    }


    public void Enter(){isExiting = false;}

    public void Exit(){isExiting = true;}

    public void onY(){}
    public void onA(){}
    public void onTrigger(){}
    public void onBumper(){}

    public void onBack()
    {
        //onBack Always goes back to drive
        player.stateMachine.ChangeState(player.driveState);
    }

}
