package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TargetingSystem;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.WithStatus;
import frc.robot.commands.swervedrive.drivebase.Drive;
import frc.robot.commands.swervedrive.drivebase.Spin;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.commands.vision.SupplyAprilTagRobotTransform;
import frc.robot.commands.vision.ReefUtil.LeftRight;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class DriveToReefRelative extends SequentialCommandGroup implements WithStatus {
    private List<Command> commandsWithStatus;
    private Transform2d targetTransform = new Transform2d();

    public DriveToReefRelative(SwerveSubsystem swerve, LeftRight side) {
        double multiplier = side == LeftRight.LEFT ? 1 : -1;
        double robotXWidth = Constants.Vision.xWidth;

        commandsWithStatus = List.of(
            new SupplyAprilTagRobotTransform((pose) -> {
                targetTransform = new Transform2d(
                    pose.getTranslation().minus(
                        new Translation2d(
                            robotXWidth, // * 5 / 8,
                            -ReefUtil.adjustY * multiplier
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
            commandsWithStatus.toArray(Command[]::new)
        );
    }

    @Override
    public boolean isRunning() {
        return commandsWithStatus.stream().map(command -> {
            if (!(command instanceof WithStatus)) { throw new Error("Command " + command.getName() + " does not implement WithStatus"); }
            return (WithStatus) command;
        }).anyMatch(WithStatus::isRunning);
    }
}