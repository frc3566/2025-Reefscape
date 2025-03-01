package frc.robot.commands;

import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.intake.PivotToSetpoint;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(Elevator elevator, Intake intake, ReefUtil.BranchLevel level) {
        double elevatorSetpoint = 0, intakeSetpoint = 0;
        switch(level) {
            case TROUGH:
                elevatorSetpoint = 1; intakeSetpoint = 2;
                break;
            case L2:
                elevatorSetpoint = 1.55; intakeSetpoint = 112;
                break; 
            case L3:
                elevatorSetpoint = 3.04; intakeSetpoint = 112;
                break;
            case L4:
                elevatorSetpoint = 1; intakeSetpoint = 2;
                break;
        }

        addCommands(
            new ElevatorToSetpoint(elevator, elevatorSetpoint),
            new PivotToSetpoint(intake, intakeSetpoint)
            // new InstantCommand(() -> intake.runIntake(false)),
            // new WaitCommand(1),
            // new InstantCommand(() -> intake.stopIntake())
            // new PivotToSetpoint(intake, 0),
            // new ElevatorToSetpoint(elevator, 1)
        );
    }

}
