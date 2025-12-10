package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommands extends Command{
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorCommands(){
        this.setName(this.getName());
        elevatorSubsystem = new ElevatorSubsystem();
        addRequirements(elevatorSubsystem);
    }

    public Command setL1Position(){
        return Commands.run(() -> 
            elevatorSubsystem.setL1Position(), elevatorSubsystem
        );
    }

    public Command setL2Position(){
        return Commands.run(() -> 
            elevatorSubsystem.setL2Position(), elevatorSubsystem
        );
    }

    public Command setL3Position(){
        return Commands.run(() -> 
            elevatorSubsystem.setL3Position(), elevatorSubsystem
        );
    }

    public Command setL4Position(){
        return Commands.run(() -> 
            elevatorSubsystem.setL4Position(), elevatorSubsystem
        );
    }

    public Command setAlgaeLowPosition(){
        return Commands.run(() -> 
            elevatorSubsystem.setAlgaeLowPosition(), elevatorSubsystem
        );
    }

    public Command setAlgaeHighPosition(){
        return Commands.run(() -> 
            elevatorSubsystem.setAlgaeHighPosition(), elevatorSubsystem
        );
    }

    public Command setBargePosition(){
        return Commands.run(() -> 
            elevatorSubsystem.setBargePosition(), elevatorSubsystem
        );
    }

    public Command setManualMode(){
        return Commands.run(() -> 
            elevatorSubsystem.setManualMode(), elevatorSubsystem
        );
    }

    public Command setIntakePosition(){
        return Commands.run(() -> 
            elevatorSubsystem.setIntakePosition(), elevatorSubsystem
        );
    }
}
