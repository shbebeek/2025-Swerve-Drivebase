package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands extends Command{
    private IntakeSubsystem intakeSubsystem;

    public IntakeCommands(){
        intakeSubsystem = new IntakeSubsystem();
    }

    public Command setRetractedPosition(){
        return Commands.run(() -> intakeSubsystem.retractIntake());
    }

    public Command setIntakingPosition(){
        return Commands.run(() -> intakeSubsystem.deployIntake());
    }

    public Command setReversePosition(){
        return Commands.run(() -> intakeSubsystem.reverseIntake());
    }
}
