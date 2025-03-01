package frc.robot.commands.swervedrive.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.drivebase.Drive;
import frc.robot.commands.swervedrive.drivebase.Spin;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.commands.vision.SupplyAprilTagRobotTransform;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToHumanRelative extends SequentialCommandGroup {
    private Transform2d targetTransform = new Transform2d();

    public DriveToHumanRelative(SwerveSubsystem swerve) {
        double robotXWidth = Constants.Vision.xWidth;

        List<Command> cmds = List.of(
            new SupplyAprilTagRobotTransform((pose) -> {
                targetTransform = new Transform2d(
                    pose.getTranslation().minus(
                        new Translation2d(
                            robotXWidth * 5 / 8,
                            0
                        ).rotateBy(pose.getRotation())
                    ),
                    pose.getRotation()
                );

                System.out.println(targetTransform);
            }, ReefUtil::getTargettingIds),
            new Drive(swerve, () -> targetTransform),
            new Spin(swerve, () -> targetTransform)
        );

        addCommands(
            cmds.toArray(Command[]::new)
        );
    }
}