package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;
import frc.robot.commands.AlignToStationCommand;

public class IntakeState extends PlayerState{
    private Command alignToStation = new AlignToStationCommand(player.swerveDriveManager, player.visionManager);

    public IntakeState(){
        super();
    }

    @Override public void Enter()
    {
        super.Enter();
        
        //player.swerveDriveManager.setSwerveToSlowDriveCommand().schedule();
        player.coralAndElevatorSubsystem.startIntake();
        System.out.println("Entered Intake State");
    }
    @Override public void Exit(){
        super.Exit();
    }

    @Override public void onTriggerUnpressed(){
        player.coralAndElevatorSubsystem.endIntake();
        player.estimatedHasCoral = true;
        player.stateMachine.ChangeState(player.driveState);
    }

    @Override public void onX(){
        alignToStation.schedule();
    }

}
