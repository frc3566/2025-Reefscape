package frc.robot.commands.auto;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TargetingSystem.ReefBranch;
import frc.robot.commands.*;
import frc.robot.commands.auto.pathplanner.TwoCoral;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.intake.PivotToSetpoint;
import frc.robot.commands.swervedrive.auto.*;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class LeftTwoCoral extends SequentialCommandGroup{
    public LeftTwoCoral(SwerveSubsystem swerve, Elevator elevator, Intake intake) {
        List<Command> cmds = List.of(
            new ParallelCommandGroup(
                new DriveToReefAbsoluteAuto(swerve,ReefUtil.Side.BARGELEFT, ReefUtil.LeftRight.LEFT),
                new ScoreCoral(elevator, intake, ReefUtil.BranchLevel.L4)
            ),
            new InstantCommand(() -> intake.runIntake(false)),
            new WaitCommand(0.5),
            new InstantCommand(() -> intake.stopIntake()),
            new ParallelCommandGroup(
                new DriveToHumanAbsolute(swerve, ReefUtil.LeftRight.LEFT),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    new GetCoral(elevator, intake)   
                )
            ),
            new InstantCommand(() -> intake.runIntake(false)),
            new WaitCommand(1.5),
            new InstantCommand(() -> intake.stopIntake()),
            new ParallelCommandGroup(
                new DriveToReefAbsoluteAuto(swerve, ReefUtil.Side.DSLEFT, ReefUtil.LeftRight.LEFT),
                new ScoreCoral(elevator, intake, ReefUtil.BranchLevel.L4))
        );
        addCommands(cmds.toArray(Command[]::new));
    }
}