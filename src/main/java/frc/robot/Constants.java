// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.05;
    public static final double SCALE_TRANSLATION = 0.8;
  }

  public static final double MAX_SPEED = Units.feetToMeters(14.77);
  public static final double kCountsPerRevolution = 4096.0;

  public static class Intake{
    public static final int kPivotMotorID = 12;
    public static final int kRollerMotorID = 13;
    public static final int kEncoderID = 4;

    public static final double kIntakePivotSpeed = 0.55;
    public static final double kIntakeRollerSpeed = 0.75;

    // just eyeballing the angles here
    public static final Rotation2d kRetractedAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kDeployedAngle = Rotation2d.fromDegrees(225);
    public static final Rotation2d kReversedAngle = Rotation2d.fromDegrees(225);

    public static final double kIntakePivotTolerance = 0.1;

    // real gear ratio from cad
    public static final double kPivotGearRatio = 42;

    public static final double kP = 0.02;
    public static final double kI = 0.0;
    public static final double kD = 0.01;

    public static final double kCurrentLimit = 40;

    public static final String kIntakeStateKey = "intakeState";
    public static final String kIntakeAtPositionKey = "intakeAtPosition";
    public static final String kIntakePivotAngle = "intakePivotAngle";
  }

  public static class Arm{
    public static final int kRollerMotorID = 10;
    public static final int kPivotMotorID = 11;
    public static final int kEncoderID = 5;
    public static double kCurrentLimit = 40.0;
    public static final double kPivotGearRatio = (5/1) * (50/16) * (84/16);

    public static String kArmAtPositionKey = "armAtPosition";
    public static String kPivotAngleKey = "pivotAngle";
    public static String kArmStateKey = "armState";
    public static String kArmSpeedKey = "armSpeed";
    public static String kArmRollerSpeedKey = "armRollerSpeed";
    public static String kArmFudgeKey = "armFudge";

    public static final double kP = 0.02;
    public static final double kI = 0.00;
    public static final double kD = 0.01;

    public static final double kArmSpeed = 0.20;
    public static final double kArmRollerSpeed = 0.25;
    // "I think we just set the starting config, looked at what the encoder said, and offset it by that much" -Landon
    public static final double kArmRealZero = 0;
    public static final double kArmPivotTolerance = 0.1;

    public static Rotation2d kIntakingAngle = Rotation2d.fromDegrees(270);
    public static Rotation2d kAlgaeAngle = Rotation2d.fromDegrees(0);
    public static Rotation2d kIdleAngle = Rotation2d.fromDegrees(90);
    public static Rotation2d kL4Angle = Rotation2d.fromDegrees(0);
    public static Rotation2d kL1Angle = Rotation2d.fromDegrees(0);
    // 45 is **about** the degrees of the reef poles for L2 and L3
    public static Rotation2d kL2L3Angle = Rotation2d.fromDegrees(45);
    // just spitballing here
    public static Rotation2d kBargeAngle = Rotation2d.fromDegrees(15);
  }

  public static class Elevator{
    public static final int kMotor1ID = 8;
    public static final int kMotor2ID = 9;
    public static double kCurrentLimit = 40.0;
    public static final double kMotorPulleyGearRatio = (60/18) * (60/18);
    public static final double kPulleyGearPitchDiameter = 20.0;
    public static final double kElevatorTolerance = 0.05;

    public static final double kP = 0.02;
    public static final double kI = 0.00;
    public static final double kD = 0.01;

    public static String kElevatorAtPositionKey = "elevatorAtPosition";
    public static String kElevatorStateKey = "elevatorState";
    public static String kElevatorFudgeKey = "elevatorFudge";
    public static String kElevatorManualModeKey = "elevatorManualMode";
    public static String kElevatorPositionKey = "elevatorPosition";

    // these are taken from 1757's elevator, need to get our own from cad
    // take distance from base of elevator to desired position
    public static final double kL4PositionBeltPosition = 52.50 * Measurements.kMetersPerInch;
    public static final double kL3PositionBeltPosition = 30 * Measurements.kMetersPerInch;
    public static final double kL2PositionBeltPosition = 15 * Measurements.kMetersPerInch;
    public static final double kL1PositionBeltPosition = 0.5 * Measurements.kMetersPerInch;
    public static final double kAlgaeHighPositionBeltPosition = 31.5 * Measurements.kMetersPerInch;
    public static final double kAlgaeLowPositionBeltPosition = 25 * Measurements.kMetersPerInch;
    public static final double kBargePositionBeltPosition = 75 * Measurements.kMetersPerInch;
    public static final double kIntakePositionBeltPosition = 41.5 * Measurements.kMetersPerInch;
  }

  public static class Drive{
    public static int kPigeonID = 0;
    public static double kP = 0.002153;
    public static double kI = 0.0;
    public static double kD = 0.53;
    public static double kAngleP = 0.01;
    public static int[] kBackLeft = {4,5,2};
    public static int[] kBackRight = {6,7,3};
    public static int[] kFrontLeft = {0,1,0};
    public static int[] kFrontRight = {2,3,1};
  }

  public static class Measurements{
    public static final double kMetersPerInch = 0.0254;
    public static final double kRadiansPerRevolution = 2 * Math.PI;
  }
}
