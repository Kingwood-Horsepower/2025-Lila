package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.CoralIntakeConstants.*;

public class CoralAndElevatorManager {
    private final Elevator elevator = new Elevator();
    private final CoralIntake coralIntake = new CoralIntake();
    private final RobotContainer robotContainer;

    public CoralAndElevatorManager(RobotContainer _robotContainer){
        robotContainer =_robotContainer;
    }
    //Data Getter Methods
    public boolean hasCoral(){
      return coralIntake.hasCoral();
    }
    public int getElevatorLevel(){
        return elevator.getElevatorLevel();
    }
    //get Commands methods


    public Command getScoreCoralComand(){
        return scoreWhenNotAtL4Command();
        //return new ConditionalCommand(scoreAtL4Command(), scoreWhenNotAtL4Command(), () -> getElevatorLevel() == 4);
    }
    private Command scoreWhenNotAtL4Command(){
        //Score Coral
        Command setCoralIntakeToLevelCommand = Commands.startEnd(()->{coralIntake.setSetPoint(0.0);
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
    private Command scoreAtL4Command(){
        Command setCoralIntakeToLevelCommand = Commands.startEnd(()->{coralIntake.setSetPoint(.23);
         }, ()->{}, coralIntake, elevator );

         Command setCoralIntakeUp = Commands.startEnd(()->{coralIntake.setSetPoint(0.20);}, ()->{}, coralIntake, elevator );

        Command outTakeCoralCommand = Commands.startEnd(() -> coralIntake.setRollerVelocity(-1), () -> coralIntake.setRollerVelocity(0), elevator, coralIntake);
        
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
              outTakeCoralCommand.withTimeout(1.5)
              .alongWith(elevatorToMaxCommand.until(() -> elevator.getIsNearSetPoint())
              .andThen(setCoralIntakeUp.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()),

              elevatorToZeroCommand.until(() -> elevator.getIsNearZero()))),   
              Commands.runOnce(this :: stowIntake, elevator, coralIntake));
    }

    public Command getIntakeCoralCommand(BooleanSupplier conditionForStoppingTheIntake, double timer){
        //Intake Coral
        Command runIntake = Commands.startEnd(()->{coralIntake.runIntake(.05, .7);}, ()->{}, coralIntake);
        Command setCor = Commands.startEnd(()->{coralIntake.setSetPoint(.26);}, ()->{}, coralIntake, elevator );
        Command el =Commands.startEnd(
            () -> {
                elevator.setElevatorLevel(0);
            }, () -> {}, coralIntake, elevator);
        Command aligCommand = robotContainer.getAlignWithStationCommand();
        return Commands.sequence(
            setCor.until(() -> coralIntake.getIsNearSetPoint() && !coralIntake.getIsNearZero()).unless( elevator :: getIsNearZero),
            el.until(elevator :: getIsNearZero),
            //aligCommand.withDeadline(runIntake.until(conditionForStoppingTheIntake).withTimeout(timer)),
            runIntake.until(conditionForStoppingTheIntake).withTimeout(timer),
            Commands.runOnce(this :: stowIntake, elevator, coralIntake));

    }
    public Command getDecrementElevatorCommand(){       
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

    
    private void stowIntake(){

        // if i am not near zero, or not set to 0, or i have coral set to down position
        if (!elevator.getIsNearZero() || elevator.getElevatorLevel() > 0 || coralIntake.hasCoral()) coralIntake.setSetPoint(.26);
        // set up
        else coralIntake.setSetPoint(0.0);
        coralIntake.setRollerVelocity(0.0);
    }

}
