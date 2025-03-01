package frc.robot.commands;

import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.intake.PivotToSetpoint;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GetCoral extends SequentialCommandGroup {
    public GetCoral(Elevator elevator, Intake intake) {
        addCommands(
            new ElevatorToSetpoint(elevator, 0.75),
            new PivotToSetpoint(intake, 56.2),
            new IntakeCoral(intake)
        );
    }

}
