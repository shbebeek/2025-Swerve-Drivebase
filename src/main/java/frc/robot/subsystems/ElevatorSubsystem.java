package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.ModifiableConstant;

public class ElevatorSubsystem extends SubsystemBase{
    public enum ElevatorState{
        Intaking,
        L4,
        L3,
        L2,
        L1,
        AlgaeHigh,
        AlgaeLow,
        Barge,
        ManualMode
    }

    private TalonFX motor1;
    private TalonFX motor2;
    private TalonFXConfiguration config;
    private CurrentLimitsConfigs currentLimit;

    PIDController pidController;
    private double kP = Constants.Elevator.kP;
    private double kI = Constants.Elevator.kI;
    private double kD = Constants.Elevator.kD;

    private ElevatorState state;
    double targetPosition = 0.0;

    private final DoublePublisher elevatorFudgePublisher;
    private final DoubleSubscriber elevatorFudgeGetter;

    private final BooleanPublisher elevatorManualModePublisher;

    private final DoublePublisher elevatorPositionPublisher;
    private final DoubleSubscriber elevatorPositionGetter;

    private final StringPublisher elevatorStatePublisher;
    private final BooleanPublisher elevatorAtPositionPublisher;

    private final ModifiableConstant l4Position = new ModifiableConstant("Elevator/L4PositionBelt",Constants.Elevator.kL4PositionBeltPosition);
    private final ModifiableConstant l3Position = new ModifiableConstant("Elevator/L3PositionBelt",Constants.Elevator.kL3PositionBeltPosition);
    private final ModifiableConstant l2Position = new ModifiableConstant("Elevator/L2PositionBelt",Constants.Elevator.kL2PositionBeltPosition);
    private final ModifiableConstant l1Position = new ModifiableConstant("Elevator/L1PositionBelt",Constants.Elevator.kL1PositionBeltPosition);
    private final ModifiableConstant algaeHighPosition = new ModifiableConstant("Elevator/AlgaeHighPositionBelt",Constants.Elevator.kAlgaeHighPositionBeltPosition);
    private final ModifiableConstant algaeLowPosition = new ModifiableConstant("Elevator/AlgaeLowPositionBelt",Constants.Elevator.kAlgaeLowPositionBeltPosition);
    private final ModifiableConstant bargePosition = new ModifiableConstant("Elevator/BargePositionBelt",Constants.Elevator.kBargePositionBeltPosition);
    private final ModifiableConstant intakePosition = new ModifiableConstant("Elevator/IntakePositionBelt",Constants.Elevator.kIntakePositionBeltPosition);
    
    public ElevatorSubsystem(){
        this.setName(this.getName());

        motor1 = new TalonFX(Constants.Elevator.kMotor1ID);
        config = new TalonFXConfiguration();
        currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = Constants.Elevator.kCurrentLimit;
        config.CurrentLimits = currentLimit;
        motor1.getConfigurator().apply(config);

        motor2 = new TalonFX(Constants.Elevator.kMotor2ID);
        config = new TalonFXConfiguration();
        currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = Constants.Elevator.kCurrentLimit;
        config.CurrentLimits = currentLimit;
        motor2.getConfigurator().apply(config);

        motor1.setNeutralMode(NeutralModeValue.Brake);
        motor2.setNeutralMode(NeutralModeValue.Brake);
        motor2.setControl(new Follower(motor1.getDeviceID(),true));
        
        this.state = ElevatorState.L1;

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(5);

        elevatorFudgePublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic(Constants.Elevator.kElevatorFudgeKey).publish();
        elevatorFudgePublisher.set(0);

        elevatorFudgeGetter = NetworkTableInstance.getDefault()
            .getDoubleTopic(Constants.Elevator.kElevatorFudgeKey).subscribe(0);
    
        elevatorManualModePublisher = NetworkTableInstance.getDefault()
            .getBooleanTopic(Constants.Elevator.kElevatorManualModeKey).publish();
        
        elevatorPositionPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic(Constants.Elevator.kElevatorPositionKey).publish();
        
        elevatorPositionGetter = NetworkTableInstance.getDefault()
            .getDoubleTopic(Constants.Elevator.kElevatorPositionKey).subscribe(Constants.Elevator.kL1PositionBeltPosition);
        
        elevatorStatePublisher = NetworkTableInstance.getDefault()
            .getStringTopic(Constants.Elevator.kElevatorStateKey).publish();
        
        elevatorAtPositionPublisher = NetworkTableInstance.getDefault()
            .getBooleanTopic(Constants.Elevator.kElevatorAtPositionKey).publish();
    }

    public boolean atPosition(){
        return (
            Math.abs(getElevatorPosition() - targetPosition)
            < Constants.Elevator.kElevatorTolerance
        );
    }

    public double getElevatorPosition(){
        // return in meters from bottom position
        return (
            motor1.getPosition().getValueAsDouble()
            / Constants.Elevator.kMotorPulleyGearRatio
            * Constants.Elevator.kPulleyGearPitchDiameter
            * Math.PI
        );
    }

    public void setElevatorMotorsAtPosition(double beltPosition){
        // get targetPosition
        targetPosition = beltPosition + elevatorFudgeGetter.get();
        // clamp targetPosition
        double minPosition = 0;
        double maxPosition = Constants.Elevator.kL4PositionBeltPosition+3*Constants.Measurements.kMetersPerInch;
        double clampedPosition = Math.max(minPosition,Math.min(targetPosition,maxPosition));
        
        // calculate setpoint
        double setpoint = (clampedPosition 
                            / (Constants.Elevator.kPulleyGearPitchDiameter * Math.PI))
                            * Constants.Elevator.kMotorPulleyGearRatio;
        
        MotionMagicDutyCycle mmRequest = new MotionMagicDutyCycle(setpoint);
        motor1.setControl(mmRequest);
    }

    public void setL4Position(){
        state = ElevatorState.L4;
        elevatorManualModePublisher.set(false);
    }

    public void setL3Position(){
        state = ElevatorState.L3;
        elevatorManualModePublisher.set(false);
    }

    public void setL2Position(){
        state = ElevatorState.L2;
        elevatorManualModePublisher.set(false);
    }

    public void setL1Position(){
        state = ElevatorState.L1;
        elevatorManualModePublisher.set(false);
    }

    public void setAlgaeHighPosition(){
        state = ElevatorState.AlgaeHigh;
        elevatorManualModePublisher.set(false);
    }

    public void setAlgaeLowPosition(){
        state = ElevatorState.AlgaeLow;
        elevatorManualModePublisher.set(false);
    }

    public void setBargePosition(){
        state = ElevatorState.Barge;
        elevatorManualModePublisher.set(false);
    }

    public void setIntakePosition(){
        state = ElevatorState.Intaking;
        elevatorManualModePublisher.set(false);
    }

    public void setManualMode(){
        state = ElevatorState.ManualMode;
        elevatorManualModePublisher.set(true);
    }

    @Override
    public void periodic(){
        switch(state){
            case ManualMode:
                setElevatorMotorsAtPosition(elevatorPositionGetter.get());
            case L4:
                setElevatorMotorsAtPosition(l4Position.value());
            case L3:
                setElevatorMotorsAtPosition(l3Position.value());
            case L2:
                setElevatorMotorsAtPosition(l2Position.value());
            case L1:
                setElevatorMotorsAtPosition(l1Position.value());
            case AlgaeHigh:
                setElevatorMotorsAtPosition(algaeHighPosition.value());
            case AlgaeLow:
                setElevatorMotorsAtPosition(algaeLowPosition.value());
            case Barge:
                setElevatorMotorsAtPosition(bargePosition.value());
            case Intaking:
                setElevatorMotorsAtPosition(intakePosition.value());
        }

        elevatorStatePublisher.set(String.valueOf(state));
        if(state != ElevatorState.ManualMode){
            elevatorPositionPublisher.set(getElevatorPosition());
        }
        elevatorAtPositionPublisher.set(atPosition());
    }
}
