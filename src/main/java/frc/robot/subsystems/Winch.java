package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase{
    private final int winchMotorID = 20;

    private final SparkMax winchMotor = new SparkMax(winchMotorID, MotorType.kBrushless);
    private final SparkMaxConfig winchMotorConfig = new SparkMaxConfig();
    //private final SparkClosedLoopController winchMotorController = winchMotor.getClosedLoopController();

    public Winch() {
        winchMotorConfig
        .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            ; // <- this semicolon is important
        winchMotor.configure(winchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runWinch(double speed) {
        winchMotor.set(speed);
    }

    public Command winchForwardCommand() {
        return Commands.runOnce(
            ()->runWinch(1), 
            this);
    }

    public Command winchReverseCommand() {
        return Commands.runOnce(
            ()->runWinch(-1), 
            this);
    }

    public Command winchStopCommand() {
        return Commands.runOnce(
            ()->runWinch(0), 
            this);
    }

}
