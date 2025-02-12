package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.util.RobotLogger;
//import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    RobotLogger logger = new RobotLogger();

    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    //private final Joystick driver2 = new Joystick(1);
    
    CommandXboxController driver = new CommandXboxController(0);

    /* Joystick Axes */
    // private final int leftThumbXID = XboxController.Axis.kLeftX.value;
    // private final int leftThumbYID = XboxController.Axis.kLeftY.value;
    // private final int rightThumbXID = XboxController.Axis.kRightX.value;
    // private final int rightThumbYID = XboxController.Axis.kRightY.value;

    // private final int leftTriggerID = XboxController.Axis.kLeftTrigger.value;
    // private final int rightTriggerID = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    // private final JoystickButton kX = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton kY = new JoystickButton(driver, XboxController.Button.kY.value);
    
    // private final JoystickButton start = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Subsystems */
    // private final Swerve s_Swerve = new Swerve();
    
    
    /* YAGSL Init */
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        "swerve/neo"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                    () -> driver.getLeftY() * -1,
                                                                    () -> driver.getLeftX() * -1)
                                                                .withControllerRotationAxis(driver::getRightX)
                                                                .deadband(Constants.Swerve.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                                                            driver::getRightX, driver::getRightY)
                                                            .headingWhile(true);


    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
    */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
        .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX())
        .withControllerRotationAxis(() -> driver.getRawAxis(2))
        .deadband(Constants.Swerve.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);


    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(() ->
                Math.sin(
                    driver.getRawAxis(2) *
                    Math.PI) *
                (Math.PI *
                2),
            () ->
                Math.cos(
                    driver.getRawAxis(2) *
                    Math.PI) *
                (Math.PI *
                2))
        .headingWhile(true);


    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@c
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        logger.logString(RobotLogger.LogLevel.Info, "RobotContainer", "Attempting to configure button bindings");
        
        /** 
         * SR: Various techniques for driving are defined as Commands below
         * */
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngleKeyboard);

        if (RobotBase.isSimulation())
        {
            /**
             * SR: Here we could use one of the commands above as the default command, but here
             * we are experimenting with a call directly to the YAGSL example's driveCommand.
             * TOD: Find out why this blocks rotation while moving with the left stick
             */
            //drivebase.setDefaultCommand(drivebase.driveCommand(
            //    () -> driveAngularVelocity.get().vxMetersPerSecond, 
            //    () -> driveAngularVelocity.get().vyMetersPerSecond, 
            //    () -> driveAngularVelocity.get().omegaRadiansPerSecond * Constants.Swerve.ANGULAR_SPEED_COEFFICIENT));

            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        } else
        {
            // When not in simulation, use the "driveFieldOrientedAngularVelocity" drive technique
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (Robot.isSimulation())
        {
            // If in simulation, reset the odometry and bind button 1 to the sysIdDriveMotor command
            driver.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driver.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
        }
        if (DriverStation.isTest())
        {
            // Various test functions
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driver.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driver.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driver.back().whileTrue(drivebase.centerModulesCommand());
            driver.leftBumper().onTrue(Commands.none());
            driver.rightBumper().onTrue(Commands.none());
        } 
        else
        {
            // When not in test mode, use these button bindings
            driver.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driver.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
            driver.b().whileTrue(
                drivebase.driveToPose(
                    new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                    );
            driver.start().whileTrue(Commands.none());
            driver.back().whileTrue(Commands.none());
            driver.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driver.rightBumper().onTrue(Commands.none());
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake)
    {
        drivebase.setMotorBrake(brake);
    }
}