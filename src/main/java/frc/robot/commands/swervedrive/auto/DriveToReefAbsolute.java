package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TargetingSystem;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.WithStatus;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.commands.vision.SupplyAprilTagFieldPose;
import frc.robot.commands.vision.ReefUtil.LeftRight;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;
import java.util.Set;

public class DriveToReefAbsolute extends SequentialCommandGroup implements WithStatus {
    private List<Command> commandsWithStatus;
    private Pose2d targetPose = new Pose2d();

    public DriveToReefAbsolute(SwerveSubsystem swerve, LeftRight side) {
        double multiplier = side == LeftRight.LEFT ? 1 : -1;
        double robotXWidth = Constants.Vision.xWidth;

        commandsWithStatus = List.of(
            new SupplyAprilTagFieldPose((pose) -> {
                targetPose = pose.transformBy(new Transform2d(
                    new Translation2d(
                        robotXWidth,
                        -ReefUtil.adjustY * multiplier
                    ), 
                    Rotation2d.k180deg
                ));

                System.out.println(targetPose);
            }, ReefUtil::getTargettingIds),
            new DeferredCommand(() -> swerve.driveToPose(targetPose), Set.of(swerve))
        );

        addCommands(
            commandsWithStatus.toArray(Command[]::new)
        );
    }

    @Override
    public boolean isRunning() {
        throw new RuntimeException("Not implemented");
    }
}