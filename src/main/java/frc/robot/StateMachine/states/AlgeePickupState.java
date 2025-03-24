package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.AlignDeAlgaeifyCommand;
import frc.robot.commands.AlignToPoseCommand;

public class AlgeePickupState extends PlayerState{

     private Command dealgeafyAlignCommand = new AlignDeAlgaeifyCommand(player.swerveDriveManager, player.visionManager);

    public AlgeePickupState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyScoringLevel();
        dealgeafyAlignCommand.schedule();
        System.out.println("Entered AlgeePickup State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onY(){
        dealgeafyAlignCommand.schedule();
        player.coralAndElevatorSubsystem.incrementDeAlgaeifyScoringLevel();
    }
    @Override public void onA(){
        if(player.coralAndElevatorSubsystem.decrementDeAlgaeifyScoringLevel())
            player.stateMachine.ChangeState(player.driveState);
    }

}
