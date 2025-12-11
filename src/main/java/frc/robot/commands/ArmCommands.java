package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommands extends Command{
    ArmSubsystem armSubsystem;
    private ElevatorCommands elevatorCommands;
    
    public ArmCommands(){
        this.setName(this.getName());
        armSubsystem = new ArmSubsystem();
        elevatorCommands = new ElevatorCommands();
        addRequirements(armSubsystem);
    }

    public Command setIdlePosition(){
        return Commands.run(() -> armSubsystem.setIdle(),armSubsystem);
    }

    public Command setIntakingPosition(){
        return Commands.run(() -> armSubsystem.setIntaking(),armSubsystem);
    }
    
    public Command setScoringPosition(){
        return Commands.run(() -> armSubsystem.setScoring(),armSubsystem);
    }

    public Command setAlgaePosition(){
        return Commands.run(() -> armSubsystem.setAlgae(),armSubsystem);
    }

    // completed intake commands
    public Command intakeCoralArmElevator(){
        return Commands.sequence(elevatorCommands.setIntakePosition(),setIntakingPosition());
    }

    public Command intakeAlgaeHighArmElevator(){
        return Commands.sequence(elevatorCommands.setAlgaeHighPosition(),setAlgaePosition());
    }

    public Command intakeAlgaeLowArmElevator(){
        return Commands.sequence(elevatorCommands.setAlgaeLowPosition(),setIntakingPosition());
    }

    public Command scoreCoral(){
        return Commands.sequence(setIntakingPosition(),intakeCoralArmElevator());
    }

    public Command scoreAlgaeBarge(){
        return Commands.sequence(elevatorCommands.setBargePosition(),setScoringPosition());
    }
}
