package frc.robot.commands.swervedrive.drivebase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WithStatus;
import frc.robot.subsystems.SwerveSubsystem;

/**
 Custom command to drive SwerveSubsystem to a target rotation, utilizing drive() from the SwerveSubsystem subsystem. 
 * Calculates the velocity of the SwerveSubsystem using PID from the supplied final pose.
 * Utilizes Translation2d with the scalar value of movement and angle of drive instead of X and Y. 
 */
public class Drive extends Command implements WithStatus {
    private SwerveSubsystem s_SwerveSubsystem;
    private Supplier<Transform2d> targetTransformSupplier;
    private Transform2d targetTransform;
    private Pose2d targetPose;

    private ProfiledPIDController driveController;

    private boolean isRunning;

    private static class DriveCommandConstants {
        public static final double kPXController = 7; //5
        public static final double kMaxSpeedMetersPerSecond = 4; //5
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    }

    public Drive(SwerveSubsystem s_SwerveSubsystem, Supplier<Transform2d> targetTransformSupplier) {
        this.s_SwerveSubsystem = s_SwerveSubsystem;
        this.targetTransformSupplier = targetTransformSupplier;

        isRunning = false;

        addRequirements(s_SwerveSubsystem);   
    }

    @Override
    public void initialize() {
        isRunning = true;

        targetTransform = targetTransformSupplier.get();
        var currentPose = s_SwerveSubsystem.getPose();
        targetPose = currentPose.transformBy(targetTransform);

        this.driveController = new ProfiledPIDController(DriveCommandConstants.kPXController, 0, 0, new TrapezoidProfile.Constraints(
            DriveCommandConstants.kMaxSpeedMetersPerSecond, DriveCommandConstants.kMaxAccelerationMetersPerSecondSquared
        ));


        driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_SwerveSubsystem.getPose();
        System.out.println("Drive: " + currentPose);
        
        /* reduce current distance (error) to 0 */
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double driveVelocityScalar = driveController.atGoal() ? 0.0 : 
            driveController.calculate(currentDistance, 0.0);

        Translation2d driveVelocity = new Translation2d(
            driveVelocityScalar, 
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
        );

        s_SwerveSubsystem.drive(driveVelocity, 0, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        s_SwerveSubsystem.drive(new Translation2d(), 0, true);
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    public boolean atGoal() {
        return isRunning && driveController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}