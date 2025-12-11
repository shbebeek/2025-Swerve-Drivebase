// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.DriveCommands.AutoAlign;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeCommands intakeCommands = new IntakeCommands();
  private final ArmCommands armCommands = new ArmCommands();
  private final ElevatorCommands elevatorCommands = new ElevatorCommands();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    // Register commands
    registerCommands();
    swerveSubsystem.setDefaultCommand(RobotBase.isSimulation() ? driveFieldOrientedDirectAngleKeyboard : driveFieldOrientedAngularVelocity);
  }

  private void registerCommands(){
    NamedCommands.registerCommand("test", Commands.print("hi there! i'm Susie"));

    NamedCommands.registerCommand("deployIntake", intakeCommands.setIntakingPosition());
    NamedCommands.registerCommand("retractIntake", intakeCommands.setReversePosition());
    NamedCommands.registerCommand("reverseIntake", intakeCommands.setReversePosition());

    NamedCommands.registerCommand("armScoring", armCommands.setScoringPosition());
    NamedCommands.registerCommand("armElevatorIntakeCoral", armCommands.intakeCoralArmElevator());
    NamedCommands.registerCommand("armElevatorIntakeAlgaeHigh", armCommands.intakeAlgaeHighArmElevator());
    NamedCommands.registerCommand("armElevatorIntakeAlgaeLow",armCommands.intakeAlgaeLowArmElevator());
    NamedCommands.registerCommand("scoreCoral",armCommands.scoreCoral());

    NamedCommands.registerCommand("elevatorL1",elevatorCommands.setL1Position());
    NamedCommands.registerCommand("elevatorL2",elevatorCommands.setL2Position());
    NamedCommands.registerCommand("elevatorL3",elevatorCommands.setL3Position());
    NamedCommands.registerCommand("elevatorL4",elevatorCommands.setL4Position());
    NamedCommands.registerCommand("elevatorAlgaeLow",elevatorCommands.setAlgaeLowPosition());
    NamedCommands.registerCommand("elevatorAlgaeHigh",elevatorCommands.setAlgaeHighPosition());
    NamedCommands.registerCommand("elevatorManualMode",elevatorCommands.setManualMode());
    NamedCommands.registerCommand("elevatorBarge",elevatorCommands.setBargePosition());
    NamedCommands.registerCommand("elevatorIntake",elevatorCommands.setIntakePosition());
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    swerveSubsystem.getSwerveDrive(),
    () -> driverController.getLeftY() * -1,
    () -> driverController.getLeftX() * -1)
    .withControllerRotationAxis(driverController::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
    driverController::getRightX,
    driverController::getRightY)
    .headingWhile(true);

  Command driveFieldOrientedDirectAngle = swerveSubsystem.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

  // sim
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX())
    .withControllerRotationAxis(() -> driverController.getRawAxis(
        2))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(() ->
          Math.sin(
              driverController.getRawAxis(
                  2) *
              Math.PI) *
          (Math.PI *
            2),
      () ->
          Math.cos(
              driverController.getRawAxis(
                  2) *
              Math.PI) *
          (Math.PI *
            2))
    .headingWhile(true)
    .translationHeadingOffset(true)
    .translationHeadingOffset(Rotation2d.fromDegrees(
        0));

  Command driveFieldOrientedDirectAngleKeyboard = swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(swerveSubsystem::exampleCondition)
        .onTrue(new AutoAlign(swerveSubsystem));

    // Schedule commands when buttons are pressed, stop sending command on release, should return to default command
    driverController.leftTrigger().whileTrue(armCommands.intakeCoralArmElevator());
    driverController.rightTrigger().whileTrue(armCommands.scoreCoral());
    driverController.b().whileTrue(elevatorCommands.setL1Position());
    driverController.a().whileTrue(elevatorCommands.setL2Position());
    driverController.y().whileTrue(elevatorCommands.setL3Position());
    driverController.x().whileTrue(elevatorCommands.setL4Position());
    driverController.leftBumper().whileTrue(armCommands.intakeAlgaeLowArmElevator());
    driverController.rightBumper().whileTrue(armCommands.intakeAlgaeHighArmElevator());
    driverController.povDown().whileTrue(armCommands.scoreAlgaeBarge());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return swerveSubsystem.getAutonomousCommand("Top Barge Auto");
  }
}
