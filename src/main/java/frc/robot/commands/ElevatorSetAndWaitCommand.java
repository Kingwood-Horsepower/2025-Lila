package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class ElevatorSetAndWaitCommand extends Command {

    private final Elevator elevator;
    
    public ElevatorSetAndWaitCommand(Elevator elevatorObject) {
        elevator = elevatorObject;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetPoint(0.0);
        elevator.setElevatorLevel(0);
    }
  
    @Override
    public boolean isFinished() {
        //if (elevator.getElevatorLevel() != 4) return true;
        return (elevator.getIsNearZero());
    }
}
