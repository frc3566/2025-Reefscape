package frc.robot.commands.vision;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Vision;

public class SupplyAprilTagPose extends Command implements WithStatus {
    private Vision s_Vision;
    
    private Consumer<Pose2d> setTargetPose;

    private int counter = 0;
    private boolean targetPoseSet = false;
    private boolean isRunning = false;

    public static final int MAX_CYCLE_COUNT = 10;

    public final List<Integer> targetIds;

    public SupplyAprilTagPose(Vision s_Vision, Pose2d currentPose, Consumer<Pose2d> setTargetPose, List<Integer> targetIds) {
        this.s_Vision = s_Vision;
        this.setTargetPose = setTargetPose;
        this.targetIds = targetIds;
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
        
        var result = Vision.Cameras.MAIN.getLatestResult();
        
        if (result.isEmpty()) {
            counter += 1;
            System.out.println("Cycle: " + counter);
            return;
        }

        s_Vision.printAllResults();

        var target = result.get().getBestTarget();
        if (!targetIds.contains(target.getFiducialId())) {
            return;
        }

        Pose2d poseToAprilTag = s_Vision.getPoseTo(target);
        System.out.println("> April Tag: " + poseToAprilTag);

        Pose2d singleDimensionTranslation = new Pose2d(
            poseToAprilTag.getTranslation().rotateBy(poseToAprilTag.getRotation().unaryMinus()),
            new Rotation2d()
        );
        System.out.println("> Translation component: " + singleDimensionTranslation);

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
