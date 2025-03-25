package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.commands.AlignDeAlgaeifyCommand;

public class AlgeePickupState extends PlayerState{

     private Command dealgeafyAlignCommand = new AlignDeAlgaeifyCommand(player.swerveDriveManager, player.visionManager);

    public AlgeePickupState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyLevel();
        dealgeafyAlignCommand.schedule();
        System.out.println("Entered AlgeePickup State");
    }
    @Override public void Exit(){
        super.Exit();
        player.coralAndElevatorSubsystem.resetAllIncrements();
    }

    @Override public void onY(){
        dealgeafyAlignCommand.schedule();
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyLevel();
    }
    @Override public void onA(){
        if(player.coralAndElevatorSubsystem.decrementDeAlgaeifyLevel())
            player.stateMachine.ChangeState(player.driveState);
    }

}
