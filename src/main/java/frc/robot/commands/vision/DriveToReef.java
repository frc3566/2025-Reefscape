package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TargetingSystem;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.WithStatus;
import frc.robot.commands.swervedrive.drivebase.Drive;
import frc.robot.commands.swervedrive.drivebase.Spin;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DriveToReef extends SequentialCommandGroup implements WithStatus {
    public enum LeftRight {
        LEFT, RIGHT
    }

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final LeftRight side;

    private List<Command> commandsWithStatus;

    double adjustY = Units.inchesToMeters(6.469);

    private Pose2d targetPose = new Pose2d();

    public DriveToReef(SwerveSubsystem swerve, VisionSubsystem vision, LeftRight side) {
        this.swerve = swerve;
        this.vision = vision;

        this.side = side;

        double multiplier = side == LeftRight.LEFT ? 1 : -1;
        double robotXWidth = Constants.Vision.xWidth;

        commandsWithStatus = List.of(
            new SupplyAprilTagPose(vision, new Pose2d(), (pose) -> {
                targetPose = new Pose2d(
                    pose.getTranslation().minus(
                        new Translation2d(
                            robotXWidth * 5 / 8,
                            -adjustY * multiplier
                        ).rotateBy(pose.getRotation())
                    ),
                    pose.getRotation()
                );

                System.out.println(targetPose);
            }, DriveToReef::getTargettingIds),
            new Drive(swerve, () -> targetPose),
            new Spin(swerve, () -> targetPose)
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

    public static List<Integer> getTargettingIds() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return List.of(17, 18, 19, 20, 21, 22);
        } else {
            return List.of(6, 7, 8, 9, 10, 11);
        }
    }
}