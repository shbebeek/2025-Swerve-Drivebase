// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.AngleOptimize;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem driveSubsystem;
  private Rotation2d targetRotation;
  PIDController rotationPID;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(SwerveSubsystem subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    targetRotation = new Rotation2d();
    rotationPID = new PIDController(Constants.Drive.kP,Constants.Drive.kI,Constants.Drive.kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var currentRotation = driveSubsystem.getRotation();
    targetRotation = Rotation2d.fromDegrees(
      Math.round(currentRotation.getDegrees()/90)*90
    );
  }

  public double rotation(){
    double optimizedDirection = AngleOptimize.optimizeAngle(driveSubsystem.getRotation(),targetRotation).getRadians();
    return rotationPID.calculate(
      driveSubsystem.getRotation().getRadians(), optimizedDirection
    );
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
      driveSubsystem.driveFieldOriented(
        rotation()
      );
    }else{
      driveSubsystem.driveFieldOriented(
        rotation()
      );
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
