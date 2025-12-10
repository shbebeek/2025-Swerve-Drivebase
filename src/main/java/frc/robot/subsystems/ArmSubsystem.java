package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.AngleOptimize;

public class ArmSubsystem extends SubsystemBase{
    public enum ArmState{
        Intaking,
        Idle,
        Scoring, // using elevator state handles l1-l4 and barge
        Algae,
    }

    private TalonFX rollerMotor;
    private TalonFX pivotMotor;
    private TalonFXConfiguration config;
    private CurrentLimitsConfigs currentLimit;


    private PIDController pidController;
    private double kP = Constants.Arm.kP;
    private double kI = Constants.Arm.kI;
    private double kD = Constants.Arm.kD;

    private DutyCycleEncoder pivotEncoder;

    private ArmState state;
    private Rotation2d targetAngle;

    private BooleanPublisher armAtPositionPublisher;
    private DoublePublisher pivotAnglePublisher;
    private StringPublisher armStatePublisher;
    private DoublePublisher armSpeedPublisher;
    private DoublePublisher armRollerSpeedPublisher;
    private DoubleSubscriber armSpeedGetter;

    private StringSubscriber elevatorPositionGetter;
    private DoublePublisher armFudgePublisher;
    private DoubleSubscriber armFudgeGetter;


    public ArmSubsystem(){
        this.setName(this.getName());
        pivotEncoder = new DutyCycleEncoder(Constants.Arm.kEncoderID);
        
        this.state = ArmState.Idle;

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(0.5);

        rollerMotor = new TalonFX(Constants.Arm.kRollerMotorID);
        config = new TalonFXConfiguration();
        currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = Constants.Arm.kCurrentLimit;
        config.CurrentLimits = currentLimit;
        rollerMotor.getConfigurator().apply(config);

        pivotMotor = new TalonFX(Constants.Arm.kPivotMotorID);
        config = new TalonFXConfiguration();
        currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = Constants.Arm.kCurrentLimit;
        config.CurrentLimits = currentLimit;
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        rollerMotor.getConfigurator().apply(config);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        armAtPositionPublisher = NetworkTableInstance.getDefault().getBooleanTopic(Constants.Arm.kArmAtPositionKey).publish();
        pivotAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kPivotAngleKey).publish();
        armStatePublisher = NetworkTableInstance.getDefault().getStringTopic(Constants.Arm.kArmStateKey).publish();
        armSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kArmSpeedKey).publish();
        armSpeedPublisher.set(Constants.Arm.kArmRollerSpeed);
        armRollerSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kArmRollerSpeedKey).publish();
        armRollerSpeedPublisher.set(Constants.Arm.kArmRollerSpeed);
        armSpeedGetter = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kArmSpeedKey).subscribe(Constants.Arm.kArmSpeed);
        armFudgeGetter = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kArmFudgeKey).subscribe(0);
        armFudgePublisher = NetworkTableInstance.getDefault().getDoubleTopic(Constants.Arm.kArmFudgeKey).publish();
        armFudgePublisher.set(0);
        elevatorPositionGetter = NetworkTableInstance.getDefault().getStringTopic(Constants.Elevator.kElevatorPositionKey).subscribe("ElevatorState.L4");
    }

    @Override
    public void periodic(){
        double armSpeed = armSpeedGetter.get();
        String elevatorState = elevatorPositionGetter.get();

        switch(state){
            case Algae:
                setPivotAngle(Constants.Arm.kAlgaeAngle);
                rollerMotor.set(-1*armSpeed);
            case Intaking:
                setPivotAngle(Constants.Arm.kIntakingAngle);
                rollerMotor.set(-1*armSpeed);
            case Idle:
                setPivotAngle(Constants.Arm.kIdleAngle);
                rollerMotor.set(0);
            case Scoring:
                rollerMotor.set(0);
                if(elevatorState == "ElevatorState.L1"){
                    setPivotAngle(Constants.Arm.kL1Angle);
                }else if(elevatorState == "ElevatorState.L2"){
                    setPivotAngle(Constants.Arm.kL2L3Angle);
                }else if(elevatorState == "ElevatorState.L3"){
                    setPivotAngle(Constants.Arm.kL2L3Angle);
                } else if(elevatorState == "ElevatorState.L4"){
                    setPivotAngle(Constants.Arm.kL4Angle);
                } else if(elevatorState == "ElevatorState.Barge"){
                    setPivotAngle(Constants.Arm.kBargeAngle);
                }
        }

        armAtPositionPublisher.set(armAtPosition());
        armStatePublisher.set(String.valueOf(state));
        // you should account for the encoder value being off in a separate function
        if(RobotBase.isSimulation()){
            pivotAnglePublisher.set(getPivotAngle());
        }else{
            pivotAnglePublisher.set(
                AngleOptimize.armAccountForSillyEncoder(getPivotAngle()));
        }
    }

    public void resetPivot(){
        double encoderFraction = AngleOptimize.armAccountForSillyEncoder(pivotEncoder.get());
        double encoderInRadians = encoderFraction * Math.PI * 2;
        double pivotMotorPosition = (
            encoderInRadians
            / Constants.Measurements.kRadiansPerRevolution
            * Constants.Arm.kPivotGearRatio
        );
        setEncoderPosition(pivotMotorPosition);
    }

    public void setEncoderPosition(double rotations){
        pivotMotor.setPosition(rotations);
    }

    public void setPivotAngle(Rotation2d rotation){
        // adding 2 rotation2ds together constrains the angle from -180 to 180
        targetAngle = Rotation2d.fromDegrees(
            rotation.getDegrees() + armFudgeGetter.get()
        );

        final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
        pivotMotor.setControl(
            motionRequest.withPosition(
            targetAngle.getDegrees()
            / Constants.Measurements.kRadiansPerRevolution
            * Constants.Arm.kPivotGearRatio)
        );
    }

    public double getPivotAngle(){
        var positionSignal = pivotMotor.getPosition();
        double positionInRotations = positionSignal.getValueAsDouble();
        return(
            positionInRotations
            / Constants.Arm.kPivotGearRatio
            * Constants.Measurements.kRadiansPerRevolution
        );
    }

    public Boolean armAtPosition(){
        return(
            Math.abs(targetAngle.getRadians() - getPivotAngle())
            < Constants.Arm.kArmPivotTolerance);
    }

    public void setIntaking(){
        state = ArmState.Intaking;
    }

    public void setAlgae(){
        state = ArmState.Algae;
    }

    public void setScoring(){
        state = ArmState.Scoring;
    }

    public void setIdle(){
        state = ArmState.Idle;
    }
}
