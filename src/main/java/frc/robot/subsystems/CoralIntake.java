package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private final int rollerMotorID = 23;
    private final int armMotorID = 24;

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
    
    private final int ARM_GEAR_RATIO = 45;
    //private final double ARM_MAX_VELOCITY = 31*ARM_GEAR_RATIO; //190 deg/sec -> rotations/min
    //private final double ARM_MAX_ACCELERATION = 633*ARM_GEAR_RATIO ; //2000 deg/sec -> rotations/min/s
    

    public CoralIntake() {
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
            .maxVelocity(1000) // takes an rpm 
            .maxAcceleration(3000) // takes an rpm/s
            .allowedClosedLoopError(0.6)
            ; // <- this semicolon is important
        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        rollerMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    @Override
    public void periodic() {
        // ill change this later
        armMotorController.setReference(setPoint*ARM_GEAR_RATIO, ControlType.kMAXMotionPositionControl);//MAXMotionPositionControl
        //rollerMotorController.setReference(velocity, ControlType.kVelocity);
        SmartDashboard.putNumber("arm encoder", armEncoder.getPosition());
        
        
    }

}
