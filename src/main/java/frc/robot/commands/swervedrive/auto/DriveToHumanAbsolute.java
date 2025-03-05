package frc.robot.commands.swervedrive.auto;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class DriveToHumanAbsolute extends SequentialCommandGroup {
    private Pose2d targetPose = new Pose2d();

    public DriveToHumanAbsolute(SwerveSubsystem swerve, ReefUtil.LeftRight side) {
        double robotXWidth = Constants.Vision.xWidth;
        List<Command> cmds = List.of(
            new InstantCommand(() -> {
                var pose = Vision.getAprilTagPose(DriveToHumanAbsolute.getTargettingId(side), new Transform2d());

                targetPose = pose.transformBy(new Transform2d(
                    new Translation2d(
                        robotXWidth,
                        0
                    ).unaryMinus().rotateBy(pose.getRotation().unaryMinus()), 
                    Rotation2d.k180deg
                ));

                System.out.println(targetPose);
            }),
            new DeferredCommand(() -> swerve.driveToPose(targetPose), Set.of(swerve))
        );

        addCommands(
            cmds.toArray(Command[]::new)
        );
    }

    public static int getTargettingId(ReefUtil.LeftRight side) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return side == ReefUtil.LeftRight.LEFT ? 13 : 12;
        } else {
            return side == ReefUtil.LeftRight.LEFT ? 1 : 2;
        }
    }
}