package frc.robot.commands.vision;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Vision;

public class SupplyAprilTagFieldPose extends Command implements WithStatus {
    private Consumer<Pose2d> setTargetPose;

    private int counter = 0;
    private boolean targetSet = false;
    private boolean isRunning = false;

    public static final int MAX_CYCLE_COUNT = 10;

    public final Supplier<List<Integer>> targetIds;

    public SupplyAprilTagFieldPose(Consumer<Pose2d> setTargetPose, Supplier<List<Integer>> targetIds) {
        this.setTargetPose = setTargetPose;
        this.targetIds = targetIds;

        System.out.println(targetIds);
    }

    @Override
    public void initialize() {
        targetSet = false;
        isRunning = true;
        counter = 0;
    }

    @Override
    public void execute() {
        if (targetSet) { return; }

        if (counter > MAX_CYCLE_COUNT) { this.cancel(); }
        
        Vision.Cameras.MAIN.updateUnreadResults();
        var result = Vision.Cameras.MAIN.getLatestResult();
        
        if (result.isEmpty()) {
            counter += 1;
            System.out.println("Cycle: " + counter);
            return;
        }

        var target = result.get().getBestTarget();
        if (!result.get().hasTargets()) { return; }
        if (!targetIds.get().contains(target.getFiducialId())) { return; }

        Pose2d aprilTagPoseOnField = Vision.getAprilTagPose(target.getFiducialId(), new Transform2d());
        System.out.println("> April Tag: " + aprilTagPoseOnField);

        setTargetPose.accept(aprilTagPoseOnField);
        targetSet = true;
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    @Override
    public boolean isFinished() {
        return targetSet;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }
}