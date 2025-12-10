package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    public enum IntakeState{
        Retracted, // intake arm up, motor at rest
        Deployed, // intake arm down, motor rolling in
        Reversed, // intake arm down, motor rolling out
    }

    private IntakeState state;
    private TalonFX pivotMotor;
    private TalonFX rollerMotor;
    private DutyCycleEncoder pivotEncoder;

    private TalonFXConfiguration configPivot;
    private PIDController pidController;

    private double pivotMotorPosition;
    private Rotation2d targetAngle;

    public IntakeSubsystem(){
        this.setName(this.getName());
        pivotEncoder = new DutyCycleEncoder(Constants.Intake.kEncoderID);

        configPivot = new TalonFXConfiguration();
        TalonFXConfiguration configRoller = new TalonFXConfiguration();

        CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = Constants.Intake.kCurrentLimit;
        currentLimit.SupplyCurrentLimitEnable = true;
        currentLimit.SupplyCurrentLimit = Constants.Intake.kCurrentLimit;
        configPivot.CurrentLimits = currentLimit;
        configRoller.CurrentLimits = currentLimit;

        pidController = new PIDController(Constants.Intake.kP,Constants.Intake.kI,Constants.Intake.kD);
        pidController.setTolerance(0.05);

        pivotMotor = new TalonFX(Constants.Intake.kPivotMotorID);
        rollerMotor = new TalonFX(Constants.Intake.kRollerMotorID);
        
        pivotMotor.getConfigurator().apply(configPivot);
        rollerMotor.getConfigurator().apply(configRoller);

        pivotMotorPosition = Units.rotationsToRadians(
            pivotEncoder.get()
            /Constants.Measurements.kRadiansPerRevolution
            * Constants.Intake.kPivotGearRatio);

        setEncoderPosition(pivotMotorPosition);
        
        this.state = IntakeState.Retracted;
        targetAngle = Constants.Intake.kRetractedAngle;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString(Constants.Intake.kIntakeStateKey, state.name());
        SmartDashboard.putBoolean(Constants.Intake.kIntakeAtPositionKey,intakeAtPosition());
        SmartDashboard.putNumber(Constants.Intake.kIntakePivotAngle,getPivotAngle());

        switch(state){
            case Retracted:
                rollerMotor.set(0.0);
                setPivotAngle(Constants.Intake.kRetractedAngle);
            
            case Deployed:
                rollerMotor.set(
                    Constants.Intake.kIntakeRollerSpeed
                );
                setPivotAngle(Constants.Intake.kDeployedAngle);

            case Reversed:
                rollerMotor.set(
                    -Constants.Intake.kIntakeRollerSpeed
                );
                setPivotAngle(Constants.Intake.kReversedAngle);
        }
    }

    public void setEncoderPosition(double rotations){
        pivotMotor.setPosition(rotations);
    }

    public void setPivotAngle(Rotation2d rotation){
        // adding 2 rotation2ds together constrains the angle from -180 to 180
        targetAngle = Rotation2d.fromDegrees(
            rotation.getDegrees()
        );

        final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
        pivotMotor.setControl(
            motionRequest.withPosition(
            targetAngle.getDegrees()
            / Constants.Measurements.kRadiansPerRevolution
            * Constants.Intake.kPivotGearRatio)
        );
    }

    public double getPivotAngle(){
        var positionSignal = pivotMotor.getPosition();
        double positionInRotations = positionSignal.getValueAsDouble();
        return(
            positionInRotations
            / Constants.Intake.kPivotGearRatio
            * Constants.Measurements.kRadiansPerRevolution
        );
    }

    public Boolean intakeAtPosition(){
        return(
            Math.abs(targetAngle.getRadians() - getPivotAngle())
            < Constants.Intake.kIntakePivotTolerance);
    }

    public void reverseIntake(){
        state = IntakeState.Reversed;
    }

    public void deployIntake(){
        state = IntakeState.Deployed;
    }

    public void retractIntake(){
        state = IntakeState.Retracted;
    }
}
