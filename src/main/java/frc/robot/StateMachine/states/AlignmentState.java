package frc.robot.StateMachine.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateMachine.PlayerState;

import frc.robot.commands.AlignToReefCommand;

public class AlignmentState extends PlayerState{
    @SuppressWarnings("unused")
    private boolean isRight = true;

    private Command alignToRightReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->true);
    private Command alignToLeftReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, ()->false);

    private Command alignToClosestReefCommand = new AlignToReefCommand(player.swerveDriveManager, player.visionManager, null);

    //private Command swerveTestCommand = new AlignCommand(player.swerveDriveManager, player.visionManager);



    public AlignmentState(){
        super();
        
        //When the alignment is not working it gives control back to the player, only x robot centric
        //player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.getSwerveDriveScoringCommand());
    }

    public void setIsRight(boolean isRight) {
        this.isRight = isRight;
    } 

    @Override public void Enter()
    {
        super.Enter();
        player.visionManager.setHighTrustInCameraRotation();
        alignToClosestReefCommand.schedule();
        //swerveTestCommand.schedule();
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        //player.swerveDriveManager.getDrivetrain().setDefaultCommand(player.swerveDriveManager.setSwerveToSlowTestDriveCommand());
        System.out.println("Entered Alignment State");
    }
    @Override public void Exit(){
        super.Exit();
        player.visionManager.setLowTrustInCameraRotation();
    }

    @Override public void onBumperPressed(){
        player.stateMachine.ChangeState(player.scoringState);
    }

    @Override public void onY(){
        player.coralAndElevatorSubsystem.incrementElevatorScoringLevel();
        alignToClosestReefCommand.schedule();

    }
    @Override public void onA(){
       ;
        if(player.coralAndElevatorSubsystem.decrementElevatorScoringLevel()){
            player.coralAndElevatorSubsystem.endIntake();
            player.stateMachine.ChangeState(player.driveState);
        }        
        else   {
            alignToClosestReefCommand.schedule();


        }
    }

    @Override public void onBack()
    {
        player.coralAndElevatorSubsystem.moveDown();
        super.onBack();
    }

    
    @Override public void onPovLeft(){
        alignToLeftReefCommand.schedule();
    }
    @Override public void onPovRight(){
        alignToRightReefCommand.schedule();
    }


}
