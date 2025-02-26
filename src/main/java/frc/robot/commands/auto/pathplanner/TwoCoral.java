package frc.robot.commands.auto.pathplanner;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.auto.DriveToHumanAbsolute;
import frc.robot.commands.swervedrive.auto.DriveToReefAbsolute;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoCoral extends SequentialCommandGroup {
    public TwoCoral(SwerveSubsystem swerve) throws FileVersionException, IOException, ParseException {
        List<Command> cmds = List.of(
            swerve.getAutonomousCommand("Start to Reef"),
            new DriveToReefAbsolute(swerve, ReefUtil.LeftRight.LEFT),
            swerve.getAutonomousCommand("Reef to Human"),
            new DriveToHumanAbsolute(swerve),
            swerve.getAutonomousCommand("Human to Reef"),
            new DriveToReefAbsolute(swerve, ReefUtil.LeftRight.RIGHT)
        );
    }
}