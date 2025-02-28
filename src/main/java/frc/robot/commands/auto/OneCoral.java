package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.intake.PivotToSetpoint;
import frc.robot.commands.swervedrive.auto.DriveToReefAbsolute;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class OneCoral extends SequentialCommandGroup{
    public OneCoral(SwerveSubsystem swerve, Elevator elevator, Intake intake) {
        List<Command> cmds = List.of(
            new DriveToReefAbsolute(swerve, ReefUtil.LeftRight.LEFT)
                .alongWith(new ElevatorToSetpoint(elevator, 0 /* TODO */))
                .alongWith(new PivotToSetpoint(intake, 0 /* TODO */))
        );
    }
}