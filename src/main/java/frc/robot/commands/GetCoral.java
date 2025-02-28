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
            new ElevatorToSetpoint(elevator, 1),
            new PivotToSetpoint(intake, 30),
            new InstantCommand(() -> intake.runIntake(true)),
            new WaitCommand(2),
            new InstantCommand(() -> intake.stopIntake())
        );
    }

}
