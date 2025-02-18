package frc.robot.commands.vision;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionSubsystem;

public class SupplyAprilTagPose extends Command implements WithStatus {
    private VisionSubsystem s_Vision;
    
    private Consumer<Pose2d> setTargetPose;

    private int counter = 0;
    private boolean targetPoseSet = false;
    private boolean isRunning = false;

    public static final int MAX_CYCLE_COUNT = 10;

    public final Supplier<List<Integer>> targetIds;

    public SupplyAprilTagPose(VisionSubsystem s_Vision, Pose2d currentPose, Consumer<Pose2d> setTargetPose, Supplier<List<Integer>> targetIds) {
        this.s_Vision = s_Vision;
        this.setTargetPose = setTargetPose;
        this.targetIds = targetIds;

        System.out.println(targetIds);

        addRequirements(s_Vision);
    }

    @Override
    public void initialize() {
        targetPoseSet = false;
        isRunning = true;
        counter = 0;
    }

    /* TODO: take currentPose into account */
    @Override
    public void execute() {
        if (targetPoseSet) { return; }

        if (counter > MAX_CYCLE_COUNT) { this.cancel(); }
        
        Vision.Cameras.MAIN.updateUnreadResults();
        var result = Vision.Cameras.MAIN.getLatestResult();
        // var result = s_Vision.getAprilTag();
        
        if (result.isEmpty()) {
            counter += 1;
            System.out.println("Cycle: " + counter);
            return;
        }

        // s_Vision.printAllResults();

        var target = result.get().getBestTarget();
        if (!result.get().hasTargets()) { return; }
        // var target = result.get();
        if (!targetIds.get().contains(target.getFiducialId())) {
            return;
        }

        Pose2d poseToAprilTag = s_Vision.getPoseTo(target);
        System.out.println("> April Tag: " + poseToAprilTag);

        setTargetPose.accept(poseToAprilTag);
        targetPoseSet = true;
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    @Override
    public boolean isFinished() {
        return targetPoseSet;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }
}
