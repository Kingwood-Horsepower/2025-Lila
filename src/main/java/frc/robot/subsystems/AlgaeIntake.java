package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.math.util.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AlgaeConstants.*;

public class AlgaeIntake extends SubsystemBase{
    //setup motors

    //DigitalInput limitSwitch = new DigitalInput(8);

    
    private final int rollerMotorID = 16;
    private final int armMotorID = 17;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    
    //look at revlib for arm motor: getForwardLimitSwitch
    //look at revlib for intake motor: getOutputCurrent

    private double setPoint = 0.0;
    private double velocity = 0.0;

    private final SparkFlex rollerMotor = new SparkFlex(rollerMotorID, MotorType.kBrushless);
    private final SparkFlexConfig rollerMotorConfig = new SparkFlexConfig();
    private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    private final int ARM_GEAR_RATIO = 135;

    public static final String algaeDownSetPointKey = "algae down point";
    public static final String algaeStoreSetPointKey = "algae store point";

    private static double algaeDownPoint = ALGAE_DOWN_POINT;
    private static double algaeStorePoint = ALGAE_STORE_POINT;


    public AlgaeIntake() {
        // continue setup
        armMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.3, 0.0, 0.0)
            .outputRange(-1, 1) //what does this do??
            .velocityFF(0) // calculated using recalc
            .maxMotion
            //idk if i want to use the units library on top of the units math util, its very verbose
            .maxVelocity(3000) // takes an rpm 
            .maxAcceleration(7000) // takes an rpm/s
            .allowedClosedLoopError(0.6)
            ; // <- this semicolon is important
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        rollerMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (!Preferences.containsKey(algaeDownSetPointKey)) {
            Preferences.setDouble(algaeDownSetPointKey, algaeDownPoint);
          }
        if (!Preferences.containsKey(algaeStoreSetPointKey)) {
          Preferences.setDouble(algaeStoreSetPointKey, algaeStorePoint);
        }
    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint FUCKING NUMBER FROM 0 TO 0.12, THE SUBSYSTEM MULTIPLIES THIS NUMBER BY THE GEAR RATIO. 
     */
    public void setSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return;
        this.setPoint = setPoint;
    }

    /**
     * Command arm to setpoint and run intake
     * 
     * @param setPoint FUCKING NUMBER FROM 0 TO 0.12, THE SUBSYSTEM MULTIPLIES THIS NUMBER BY THE GEAR RATIO. 
     * @param velocity velocity of the intake motor, idk what the range is
     */
    public void runIntake(double setPoint, double velocity) {
        setSetPoint(setPoint);
        this.velocity = velocity;
        rollerMotor.set(velocity);

    }

    public void loadPreferences() {
        algaeDownPoint = Preferences.getDouble(algaeDownSetPointKey, algaeDownPoint);
        algaeStorePoint = Preferences.getDouble(algaeStoreSetPointKey, algaeStorePoint);
    }

    public Command intake() {
        return Commands.startEnd(
            () -> runIntake(algaeDownPoint, -1.0),
            () -> runIntake(algaeStorePoint, 0.0), 
            this);
    }

    public Command score() {
        return Commands.startEnd(
            () -> runIntake(algaeDownPoint, 1.0),
            () -> runIntake(0.0, 0.0), 
            this);
    }


    @Override
    public void periodic() {
        // ill change this later
        armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl);//MAXMotionPositionControl
        //rollerMotorController.setReference(velocity, ControlType.kVelocity);
        
        //SmartDashboard.putBoolean("limit", limitSwitch.get());
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        
        
    }

    

}
