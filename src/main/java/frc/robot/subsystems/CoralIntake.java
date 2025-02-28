package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CoralIntakeConstants.*;

public class CoralIntake extends SubsystemBase {
    private final DigitalInput IRsensor = new DigitalInput(9); // false = broken

    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

    private final SparkMax armMotor = new SparkMax(armMotorID, MotorType.kBrushless);
    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder altEncoder = armMotor.getAlternateEncoder();


    private double setPoint = 0; //armStowPositionPerpendicular
    private double velocity = 0.0;
    private boolean hasCoral = false;

    private final SparkMax rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushless);
    private final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController rollerMotorController = rollerMotor.getClosedLoopController();
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    
    private final int ARM_GEAR_RATIO = 3; // this is the sprocket gear ratio now

    //private final ArmFeedforward feedforward = new ArmFeedforward(0, kG, 0, 0);

    public CoralIntake() {

        // continue setup
        //altEncoder.setPosition(armStowPositionPerpendicular);
        SmartDashboard.putString("did i setPosition?", "a");
        

        armMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true) 
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(.5, 0.0, 0.5)
            .outputRange(-1, 1) //what does this do??
            //.velocityFF(0) // calculated using recalc
            .maxMotion
            //idk if i want to use the units library on top of the units math util, its very verbose
            .maxVelocity(266) // takes an rpm 
            .maxAcceleration(500) // takes an rpm/s
            .allowedClosedLoopError(0.02)
            ; // <- this semicolon is important
        armMotorConfig
            .alternateEncoder
            .countsPerRevolution(8192)
            .setSparkMaxDataPortConfig()
            .inverted(false)
            ;
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        rollerMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(true);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint rotations of the big sprocket
     */
    public void setSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return;
        this.setPoint = setPoint;
    }

    public void setRollerVelocity(double velocity) {
        this.velocity = velocity;
        rollerMotor.set(velocity);
    }

    /**
     * Command arm to setpoint and run intake
     * 
     * @param setPoint rotations of the big sprocket, ~ 0, 0.3
     * @param velocity velocity of the intake motor,   -1, 1 
     */
    public void runIntake(double setPoint, double velocity) {
        setSetPoint(setPoint);
        setRollerVelocity(velocity);
    }


    public boolean getIsNearSetPoint() {
        double tolerance = 0.1; // in small sprocket rotations
        double currPosition = altEncoder.getPosition();
        double targetPosition = setPoint*ARM_GEAR_RATIO; 
        if ((currPosition > targetPosition - tolerance) && (currPosition < targetPosition + tolerance)) return true;
        return false;
    }

    public boolean getIsNearZero() {
        double tolerance = 0.1; // in small sprocket rotations
        double currPosition = altEncoder.getPosition(); 
        if ((currPosition > 0 - tolerance) && (currPosition < 0 + tolerance)) return true;
        return false;
    }

    public boolean runningHighAmps(){
        double threshold = 10;
        return (rollerMotor.getOutputCurrent() > threshold);
    }
    
    public boolean hasCoral()
    {
        return hasCoral;
    }

    @Override
    public void periodic() {
        // ill change this later
        double feedforward = 0;
        //Math.cos(setPoint*2*Math.PI)*kG;
        armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward);//MAXMotionPositionControl
        hasCoral = !IRsensor.get(); 

        SmartDashboard.putBoolean("is at setpoint",getIsNearSetPoint());
        SmartDashboard.putBoolean("arm is not near zero", !getIsNearZero());
        SmartDashboard.putNumber("arm setPoint", setPoint*ARM_GEAR_RATIO);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("arm alt encoder", altEncoder.getPosition());
        SmartDashboard.putNumber("roller amps", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasCoral", hasCoral);
                
        
    }
    

}
