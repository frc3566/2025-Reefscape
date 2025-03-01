// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GetCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.intake.PivotToSetpoint;
import frc.robot.commands.swervedrive.auto.DriveToReefAbsolute;
import frc.robot.commands.swervedrive.auto.DriveToReefRelative;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.Drive;
import frc.robot.commands.swervedrive.drivebase.Spin;
import frc.robot.commands.vision.ReefUtil;
import frc.robot.commands.vision.SupplyAprilTagRobotTransform;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;
import java.util.function.BooleanSupplier;

import org.dyn4j.geometry.Transform;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator();
  private final Algae algae = new Algae();
  private final Intake intake = new Intake();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY(),
      () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // TELEOP HERE
      // TODO: Refactor with suppliers?

      /*
       * Keyboard simulation controls:
       * z -> a()
       * x -> b()
       * c -> x()
       * v -> y()
       */

      // driverXbox.y().onTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.a().onTrue(drivebase.sysIdAngleMotorCommand());

      driverXbox.b().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      
      /* Buttons - Climb / Gyro */

      // driverXbox.a().whileTrue(
      //   new Drive(drivebase, () -> new Transform2d(new Translation2d(1, 0), new Rotation2d()))
      // );

      // driverXbox.y().whileTrue(
      //   new Spin(drivebase, () -> new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)))
      // );

      // driverXbox.a().whileTrue(
      //   new Spin(drivebase, () -> new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)))
      // );

      // driverXbox.y().whileTrue(
      //   new SequentialCommandGroup(
      //     new InstantCommand(() -> {
      //       drivebase.resetOdometry(new Pose2d());
      //     }),
      //     drivebase.driveToPose(new Pose2d(new Translation2d(1, 0), new Rotation2d()))
      //   )
      // );
      driverXbox.y().whileTrue(drivebase.sysIdDriveMotorCommand()); //Drive straight
      driverXbox.a().whileTrue(drivebase.sysIdAngleMotorCommand()); //Angle motor
      // driverXbox.y().whileTrue(new DriveToReefRelative(this.drivebase, ReefUtil.LeftRight.RIGHT));
      // driverXbox.a().whileTrue(new DriveToReefRelative(this.drivebase, ReefUtil.LeftRight.LEFT));
      // driverXbox.a().whileTrue(new DriveToReefAbsolute(this.drivebase, ReefUtil.LeftRight.LEFT));

      driverXbox.x().whileTrue(new InstantCommand(() -> {
        System.out.println("pressed x");
        Vision.Cameras.MAIN.updateUnreadResults();
        var result = Vision.Cameras.MAIN.getCamera().getLatestResult();
        System.out.println(result.hasTargets() ? result.getBestTarget() : null);
        // Vision.Cameras.MAIN.getBestResult()
        //   .map(e -> (e.hasTargets() ? e.getBestTarget() : null))
        //   .map(Vision::getRobotRelativeTransformTo)
        //   .ifPresent(System.out::println);
      }).repeatedly());

      /* Bumpers - Pivot */
      driverXbox.leftBumper().onTrue(new InstantCommand(() -> intake.set(false)));
      driverXbox.leftBumper().onFalse(new InstantCommand(() -> intake.stopPivot()));
      
      driverXbox.rightBumper().onTrue(new InstantCommand(() -> intake.set(true)));
      driverXbox.rightBumper().onFalse(new InstantCommand(() -> intake.stopPivot()));
 
      /* Triggers - Elevator */
      driverXbox.leftTrigger().onTrue(new InstantCommand(() -> elevator.up()));
      driverXbox.leftTrigger().onFalse(new InstantCommand(() -> elevator.stop()));

      driverXbox.rightTrigger().onTrue(new InstantCommand(() -> elevator.down()));
      driverXbox.rightTrigger().onFalse(new InstantCommand(() -> elevator.stop()));

      /* DPad - Coral / Algae */
      driverXbox.povUp().onTrue(new InstantCommand(() -> intake.runIntake(false))); //coral out
      driverXbox.povUp().onFalse(new InstantCommand(() -> intake.stopIntake()));

      driverXbox.povDown().onTrue(new InstantCommand(() -> intake.runIntake(true))); // coral in
      driverXbox.povDown().onFalse(new InstantCommand(() -> intake.stopIntake()));

      driverXbox.povLeft().onTrue(new InstantCommand(() -> algae.out())); 
      driverXbox.povLeft().onFalse(new InstantCommand(() -> algae.stop()));

      driverXbox.povRight().onTrue(new InstantCommand(() -> algae.in())); 
      driverXbox.povRight().onFalse(new InstantCommand(() -> algae.stop()));

      driverXbox.povUpRight().whileTrue(new ElevatorToSetpoint(elevator, 0.66));
      driverXbox.povUpLeft().whileTrue(new GetCoral(elevator, intake));
      driverXbox.povDownLeft().whileTrue(new ScoreCoral(elevator, intake, ReefUtil.BranchLevel.L2));
      driverXbox.povDownRight().whileTrue(new ScoreCoral(elevator, intake , ReefUtil.BranchLevel.L3));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
    return new Drive(drivebase, () -> new Transform2d(new Translation2d(2, 0), new Rotation2d()));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void updateSimulation() {
    if (!Robot.isSimulation()) { return; }
    this.drivebase.vision.visionSim.update(this.drivebase.getPose());
  }
}