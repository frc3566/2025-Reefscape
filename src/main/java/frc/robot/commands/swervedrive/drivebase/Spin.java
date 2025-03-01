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
 * Custom command to rotate SwerveSubsystem to a target rotation, utilizing drive() from the SwerveSubsystem subsystem. 
 * Calculates the velocity of rotation using PID from the supplied final pose.
 */
public class Spin extends Command implements WithStatus {
    private SwerveSubsystem s_SwerveSubsystem;
    private Supplier<Transform2d> targetTransformSupplier;
    private Transform2d targetTransform;
    private double targetAngle;

    private ProfiledPIDController thetaController;

    private boolean isRunning;

    private static class SpinCommandConstants {
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.5 * Math.PI; 
        public static final double kMaxAngularSpeedRadiansPerSecondSquared =  2.25 * Math.PI; 
        public static final double kPThetaController = 5; //6

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    public Spin(SwerveSubsystem s_SwerveSubsystem, Supplier<Transform2d> targetTransformSupplier) {
        this.s_SwerveSubsystem = s_SwerveSubsystem;
        this.targetTransformSupplier = targetTransformSupplier;

        isRunning = false;

        addRequirements(s_SwerveSubsystem);   
    }

    @Override
    public void initialize() {
        isRunning = true;
        
        targetTransform = targetTransformSupplier.get();
        double currentRadians = s_SwerveSubsystem.getPose().getRotation().getRadians();
        targetAngle = targetTransform.getRotation().getRadians() + currentRadians;

        this.thetaController = new ProfiledPIDController(
            SpinCommandConstants.kPThetaController, 0, 0, SpinCommandConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        thetaController.reset(currentRadians);
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_SwerveSubsystem.getPose();
        System.out.println("Spin: " + currentPose);

        double thetaVelocity = thetaController.atGoal() ? 0.0 : 
            thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle);

        s_SwerveSubsystem.drive(new Translation2d(), thetaVelocity, true);
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
        return isRunning && thetaController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}