package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase{

    // initialize important objects for the elevator motors and limit switch. 
    private final DigitalInput limitSwitch = new DigitalInput(8);
    // only turns true after being true for a second, but turns false immediately on false (kRising)
    private final Debouncer limitSwtichDebouncer = new Debouncer(1, Debouncer.DebounceType.kRising);

    private final int leadMotorID = 21;
    private final int followMotorID = 19;

    private final SparkMax leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
    private final SparkMaxConfig leadMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder leadEncoder = leadMotor.getEncoder();

    private final SparkMax followMotor = new SparkMax(followMotorID, MotorType.kBrushless);
    private final SparkMaxConfig followMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder followEncoder = followMotor.getEncoder();

    // velocity and acceleration constraints of the PID controller below. 
    // The PID controller is what calculates elevator speed in periodic()
    private final TrapezoidProfile.Constraints ELEVATOR_MOTOR_ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(400, 400);
    private final ProfiledPIDController elevatorController = new ProfiledPIDController(1, 0, 0, ELEVATOR_MOTOR_ROTATION_CONSTRAINTS);


    // setpoint of the elevator, the elevator is driven to this value on every run of periodic()
    private double setPoint = 0.0;
    // used for the limit switch, not working
    private boolean isZerod = false; 
    private boolean wasZerod = false;
    private boolean elevatorOverriden = false;
    
    
    // Simulation stuff, not working well for now

    // create a motor model and sparkMaxSim object with the motor model
    private final DCMotor elevatorDCMotor = DCMotor.getNEO(1); // this should be 2 since 2 motors are used
    private final SparkMaxSim leadMotorSim = new SparkMaxSim(leadMotor, elevatorDCMotor);

    // create an elevator simulation with our elevator's characteristics
    private final ElevatorSim elevatorSim = new ElevatorSim(
        elevatorDCMotor, 
        ELEVATOR_GEAR_RATIO, 
        9, 
        0.0152, 
        0.1, 
        11.6, 
        false, 
        0, 
        0.01,
        0
    );
    
    // create mechanisms, read wpilib for this.
    private final Mechanism2d mech = new Mechanism2d(50, 100);
    private final MechanismRoot2d rootMech = mech.getRoot("base", 10, 0);
    // create ligament mechanism that represents the elevator 
    private final MechanismLigament2d elevatorMech2d =
      rootMech.append(
          new MechanismLigament2d("elevator", 0.1*20, 90) // 20 is pixels per meter
          );
    // our pid code relies on the outputs of the encoder, so we must simulate it aswell
    private final SparkRelativeEncoderSim leadEncoderSim = leadMotorSim.getRelativeEncoderSim();
    

    public Elevator() {
        /*  revlib has the ability to configure the sparkmaxes run the Profiled PID controllers in the sparkmax hardware
            but I have had unsolvable issues with the motion being shaky and not smooth. 
            I would advise against using it unless it has been improved for the 2026 season
         */

        // configure rev motors with simple configurations.
        leadMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false); 
        leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .follow(leadMotor, true);
        followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // create dashboard override buttons. They take a name and a command that is run on press. Right now, these overrides are unusable
        SmartDashboard.putData("override elevator up", OverrideElevatorUp());
        SmartDashboard.putData("override elevator down", OverrideElevatorDown());
    }

    /**
     * Command arm to setpoint
     * 
     * @param setPoint double that represents distance of the middle stage to its bottom 
     */
    public void setSetPoint(double setPoint) {
        System.out.println("elevator setpoint set to: " + Double.toString(setPoint));
        this.setPoint = setPoint;
    }


    public double getLeadEncoderPosition() {
        if(Robot.isSimulation()) return leadEncoderSim.getPosition();
        else return leadEncoder.getPosition();
    }

    /**
     * Determines if the elevator is near (2 motor rotations of tolerance) the current setPoint
     */    
    public boolean getIsNearSetPoint() {
        double tolerance = 2;  // in encoder rotations
        double currPosition = getLeadEncoderPosition(); 
        double targetPosition = setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS;
        return Math.abs(currPosition-targetPosition) < tolerance;
    }

    public boolean getIsLimitSwitchZerod() {
        return isZerod;
    }

    public void resetEncoders() {
        leadEncoder.setPosition(0.0);
        followEncoder.setPosition(0.0);
    }

    /**
     * Move the Elevator to the setPoint and return when finished
     * 
     * @param setPoint setpoint to move the arm to
     */    
    public Command moveToSetPoint(double setPoint) {
        if (setPoint == this.setPoint) return Commands.none();
        return Commands.runOnce(()-> setSetPoint(setPoint), this);//.until(() -> getIsNearSetPoint());
    }

    // deprecated. A command like this can be used to have the command only end when it is near the setPoint it is initially commanded to go to
    // public Command moveUntilAtSetPoint(double setPoint) {
    //     if (setPoint == this.setPoint) return Commands.none();
    //     return new FunctionalCommand(
    //         () -> {
    //             setSetPoint(setPoint);
    //         }, 
    //         ()->{}, 
    //         t->{},
    //         () -> getIsNearSetPoint(), 
    //         this);
    // }


    // these are bad
    public Command OverrideElevatorUp() {
        return Commands.startEnd(
            ()->{
                elevatorOverriden = true;
                leadMotor.set(0.2);
            }, 
            ()->{
                leadMotor.set(0);
                elevatorOverriden = false;
            }, this);
    }

    public Command OverrideElevatorDown() {
        return Commands.startEnd(
            ()->{
                elevatorOverriden = true;
                leadMotor.set(-0.2);
            }, 
            ()->{
                leadMotor.set(0);
                elevatorOverriden = false;
            }, this);
    }

    @Override
    public void periodic() {

        //leadMotorController.setReference(setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS, ControlType.kMAXMotionPositionControl); // the rev version of PID. Had problems as discussed above
        if (!elevatorOverriden)leadMotor.setVoltage(elevatorController.calculate(getLeadEncoderPosition(), setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS));
        // only tells that the elevator hit the limit switch if activated for one second, and only runs command on rising edge
        isZerod = limitSwtichDebouncer.calculate(limitSwitch.get());
        if (isZerod && !wasZerod) { // if the limit switch has hit, and it havent been zeroed yet
            System.out.println("zeroed");
            resetEncoders();
        }
        wasZerod = isZerod;

        SmartDashboard.putNumber("lead elevator encoder", leadEncoder.getPosition());
        SmartDashboard.putNumber("follow elevator encoder", followEncoder.getPosition());
        SmartDashboard.putNumber("elevator setpoint", setPoint*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS);
        SmartDashboard.putBoolean("is zeroed limitwsitch", isZerod);
        SmartDashboard.putBoolean("elevator is near setpoint", getIsNearSetPoint());
        SmartDashboard.putNumber("elevator motor current", leadMotor.getOutputCurrent());

    }

    @Override 
    public void simulationPeriodic() {
        if (elevatorController.getP() != 90) elevatorController.setP(90);

        //m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        // calculate the mechanism's new velocity with the motor sim
        elevatorSim.setInput(leadMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        //update on standard loop time
        elevatorSim.update(0.02);

        // setting the motor sim to the mechanism's current calculated velocity
        // Iterate the elevator and arm SPARK simulations
        double inchesPerMeter = 39.37;
        
        leadMotorSim.iterate(elevatorSim.getVelocityMetersPerSecond()*inchesPerMeter*ELEVATOR_INCHES_TO_MOTOR_REVOLUTIONS, RobotController.getBatteryVoltage(), 0.02);
        // meters   inches   revolutions 
        // ------ * ------ * ----------- * seconds = revolutions of the motor
        // second   meter    inch      
        
        // simulating the power draw from the battery by this mechanism
        RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
        
        // set the encododer sim 
        leadEncoderSim.setPosition(leadMotorSim.getPosition());
        // modify the mechanism
        elevatorMech2d.setLength(.1+elevatorSim.getPositionMeters()*10);
        SmartDashboard.putData("elevator please", mech);
        
    }
}