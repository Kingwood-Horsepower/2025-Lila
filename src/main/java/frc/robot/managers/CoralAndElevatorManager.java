package frc.robot.managers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
      return coralIntake.hasCoral;
    }

    //get Commands methods
    public Command getScoreCoralComand(BooleanSupplier conditionForStoppingTheIntake){
        //Score Coral
        return scoreBelowL4Command(conditionForStoppingTheIntake);
    
        //return new ConditionalCommand(scoreAtL4Command(), scoreBelowL4Command(conditionForStoppingTheIntake), ()  -> elevator.getElevatorLevel() ==4);
    }
    // private Command scoreAtL4Command(){
    //     Command outTakeCoralCommand = Commands.startEnd(() -> coralIntake.setRollerVelocity(-1), () -> coralIntake.setRollerVelocity(0));
    //     return outTakeCoralCommand.withTimeout(2.5);
    // }
  private Command scoreAtL4Command(){
      Command setCoralIntakeToLevelCommand = Commands.startEnd(()->{coralIntake.setSetPoint(0.22);
       }, ()->{}, coralIntake);

       Command setCoralIntakeUp = Commands.startEnd(()->{coralIntake.setSetPoint(0.22);}, ()->{}, coralIntake);

      Command outTakeCoralCommand = Commands.startEnd(() -> coralIntake.setRollerVelocity(-1), () -> coralIntake.setRollerVelocity(0));
      
      Command elevatorToMaxCommand = Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(5);
            }, () -> {}, elevator);

      Command elevatorToZeroCommand = Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(0);
            }, () -> {}, elevator);

        return Commands.sequence(
            setCoralIntakeToLevelCommand.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()),
            new WaitCommand(.3),
            //Fancy ahahah stuff
            outTakeCoralCommand.withTimeout(2.5)
            .alongWith(elevatorToMaxCommand.until(() -> elevator.getIsNearSetPoint())
            .andThen(setCoralIntakeUp.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()),
            new WaitCommand(.5),
            elevatorToZeroCommand.until(() -> elevator.getIsNearZero()))),   
            Commands.runOnce(this :: stowIntake, elevator, coralIntake));
  }

    public Command scoreBelowL4Command(BooleanSupplier conditionForStoppingTheIntake){
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
              outTakeCoralCommand.until(conditionForStoppingTheIntake).withTimeout(1.5), //Timeout is for the autoroutine
              elevatorToZeroCommand.until(() -> elevator.getIsNearZero()),
              Commands.runOnce(this :: stowIntake, elevator, coralIntake)
          );
    }

    public Command getIntakeCoralCommand(BooleanSupplier conditionForStoppingTheIntake){
        //Intake Coral
        Command runIntake = Commands.startEnd(()->{coralIntake.runIntake(.04, .7);}, ()->{}, coralIntake);
        Command setCoralDown = Commands.startEnd(()->{coralIntake.setSetPoint(0.26);}, ()->{}, coralIntake, elevator );
        Command setElevatorLevelOne =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(1);
            }, () -> {}, coralIntake, elevator);


        Command setElevatorLevelZero =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(0);
            }, () -> {}, elevator);

        return Commands.sequence(
            setCoralDown.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()).unless( elevator :: getIsNearZero),
            //setElevatorLevelOne.until(elevator :: getIsNearSetPoint),
            runIntake.until(conditionForStoppingTheIntake),       
            setElevatorLevelZero.until(elevator :: getIsNearSetPoint),
            Commands.runOnce(this :: stowIntake, coralIntake, elevator));

    }
    public Command getMoveRollersCommand(){
        return Commands.startEnd(()->{coralIntake.runIntake(.00, .7);}, this :: stowIntake);
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
    private void stopIntake(){
        coralIntake.setSetPoint(0.0);
        coralIntake.setRollerVelocity(0.0);
    }
    private void stowIntake(){
        

        // if i am not near zero, or not set to 0, or i have coral set to down position
        if (!elevator.getIsNearZero() || elevator.getElevatorLevel() > 0 || coralIntake.hasCoral) {
            coralIntake.setSetPoint(.26);
            System.out.println("stow down");
        }
        // set up
        else {
            coralIntake.setSetPoint(0.0);
            System.out.println("stow up");
        }
        System.out.println("stop rollers");
        coralIntake.setRollerVelocity(0.0);
    }

    public CoralIntake getCoralIntake() {
        return coralIntake;
    }

    public Elevator getElevator() {
        return elevator;
    }


}
