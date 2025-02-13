package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class CoralIntakeSetAndWaitCommand extends Command {
    private final CoralIntake coralIntake;
    private final Elevator elevator;
    
    public CoralIntakeSetAndWaitCommand(CoralIntake coralIntakeObject, Elevator elevatorObject) {
        coralIntake = coralIntakeObject;
        elevator = elevatorObject;
        addRequirements(coralIntake, elevator);
    }

    @Override
    public void initialize() {
        if (elevator.getElevatorLevel() == 4) {
            coralIntake.setSetPoint(0.07);
            }
            else coralIntake.setSetPoint(0.07); // 0
    }
  
    @Override
    public boolean isFinished() {
        //if (elevator.getElevatorLevel() != 4) return true;
        return (coralIntake.getIsAtSetPoint() && !coralIntake.getIsNearZero());
    }
}
