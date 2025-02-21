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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TargetingSystem;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.WithStatus;
import frc.robot.commands.swervedrive.drivebase.Drive;
import frc.robot.commands.swervedrive.drivebase.Spin;
import frc.robot.commands.vision.ReefUtils.LeftRight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class DriveToReefAbsolute extends SequentialCommandGroup implements WithStatus {
    private final SwerveSubsystem swerve;
    private final LeftRight side;

    private List<Command> commandsWithStatus;

    final double adjustY = Units.inchesToMeters(6.469);

    private Pose2d targetPose = new Pose2d();

    public DriveToReefAbsolute(SwerveSubsystem swerve, LeftRight side) {
        this.swerve = swerve;
        this.side = side;

        double multiplier = side == LeftRight.LEFT ? 1 : -1;
        double robotXWidth = Constants.Vision.xWidth;

        commandsWithStatus = List.of(
            new SupplyAprilTagFieldPose((pose) -> {
                targetPose = pose.transformBy(new Transform2d(
                    new Translation2d(
                        robotXWidth,
                        -adjustY * multiplier
                    ).unaryMinus().rotateBy(pose.getRotation().unaryMinus()), 
                    Rotation2d.k180deg
                ));

                System.out.println(targetPose);
            }, ReefUtils::getTargettingIds),
            new DeferredCommand(() -> swerve.driveToPose(targetPose), Set.of(swerve))
        );

        addCommands(
            commandsWithStatus.toArray(Command[]::new)
        );
    }

    @Override
    public boolean isRunning() {
        throw new RuntimeException("Not implemented");
        // return commandsWithStatus.stream().map(command -> {
        //     if (!(command instanceof WithStatus)) { throw new Error("Command " + command.getName() + " does not implement WithStatus"); }
        //     return (WithStatus) command;
        // }).anyMatch(WithStatus::isRunning);
    }

    public static List<Integer> getTargettingIds() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return List.of(17, 18, 19, 20, 21, 22);
        } else {
            return List.of(6, 7, 8, 9, 10, 11);
        }
    }
}