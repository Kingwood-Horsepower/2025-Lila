package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class CoralAndElevatorManager {
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake();


    public CoralAndElevatorManager(){

    }
    //Data Getter Methods
    public boolean hasCoral(){
      return coralIntake.hasCoral();
    }

    public void stowIntake(){
        // if i am not near zero, or not set to 0, or i have coral set to down position
        if (!elevator.getIsNearZero() || elevator.getElevatorLevel() > 0 || coralIntake.hasCoral()) coralIntake.setSetPoint(.26);
        // set up
        else coralIntake.setSetPoint(0.0);
        coralIntake.setRollerVelocity(0.0);
    }

    //get Commands methods


    public Command getScoreCoralComand(){
          //Score Coral
          Command setCoralIntakeToLevelCommand = Commands.startEnd(()->{
            if(elevator.getElevatorLevel() == 4) {
                coralIntake.setSetPoint(0.25);
            } else{coralIntake.setSetPoint(0.26);}
         }, ()->{}, coralIntake, elevator );

          Command outTakeCoralCommand = Commands.startEnd(() -> coralIntake.setRollerVelocity(-1), () -> coralIntake.setRollerVelocity(0), elevator, coralIntake);

          Command elevatorToZeroCommand = Commands.startEnd(
              () -> {
                  elevator.setElevatorLevel(0);
              }, () -> {}, coralIntake, elevator);
  
          return Commands.sequence(
              setCoralIntakeToLevelCommand.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()),
              new WaitCommand(.3),
              outTakeCoralCommand.withTimeout(1.5),
              elevatorToZeroCommand.until(() -> elevator.getIsNearZero()),
              Commands.runOnce(this :: stowIntake, elevator, coralIntake)
          );
    }

    public Command getIntakeCoralCommand(BooleanSupplier conditionForStoppingTheIntake){
        //Intake Coral
        Command runIntake = Commands.startEnd(()->{coralIntake.runIntake(.05, .7);}, ()->{}, coralIntake);
                // intakeCoral = Commands.startEnd(
                //     () -> {
                //         coralIntake.runIntake(.07, .7);
                //         elevator.setElevatorLevel(0);
                //     },
                //     () -> coralIntake.stowIntake(),
                //     coralIntake, elevator).until(()-> elevator.getIsNearSetPoint() && coralIntake.getIsNearSetPoint());
        Command setCor = Commands.startEnd(()->{coralIntake.setSetPoint(0.26);}, ()->{}, coralIntake, elevator );
        Command el =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(0);
            }, () -> {}, coralIntake, elevator);
        return Commands.sequence(
            setCor.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()).unless( elevator :: getIsNearZero),
            el.until(elevator :: getIsNearZero),
            runIntake.until(conditionForStoppingTheIntake),
            Commands.runOnce(this :: stowIntake, elevator, coralIntake));

    }
    public Command getDecrementElevatorCommand(){
                //increment elevator
                
        
                //decrement elevator 
                return Commands.startEnd(
                    () -> {
                        elevator.decrementElevatorLevel();
                        elevator.setElevatorLevel();
                        stowIntake();
                    },
                    () -> {},
                    coralIntake, elevator).until(elevator::getIsNearSetPoint);
    }

    public Command getIncrementElevatorCommand(){
        return Commands.startEnd(
            () -> {
                elevator.incrementElevatorLevel();
                elevator.setElevatorLevel();
                stowIntake();
            },
            () -> {},
            coralIntake, elevator).until(elevator::getIsNearSetPoint);
    }
    public Command getSetElevatorCommand(int level){
        return Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(level);
                stowIntake();
            },
            () -> {});
    }

}
