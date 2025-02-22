// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SysIdPivot;
import frc.robot.subsystems.SysIdSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotRoutine {
  // The robot's subsystems
  private final SysIdSwerve m_drive = new SysIdSwerve();
  private final SysIdPivot m_Pivot = new SysIdPivot();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(0);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotRoutine() {
    configureBindings();
  }

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Control the drive with split-stick arcade controls
    // m_drive.setDefaultCommand(
    //     // m_drive.drive(
    //     //     () -> -m_driverController.getLeftY(), () -> -m_driverController.getRightX()));
    //     new TeleopSwerve(
    //           m_drive,
    //           () -> -m_driverController.getRawAxis(translationAxis),
    //           () -> -m_driverController.getRawAxis(strafeAxis), 
    //           () -> -m_driverController.getRawAxis(rotationAxis), 
    //           () -> m_driverController.y().getAsBoolean()
    //       )
    //     );

    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
    // m_driverController.a().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_driverController.a().whileTrue(m_Pivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.b().whileTrue(m_Pivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.x().whileTrue(m_Pivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.y().whileTrue(m_Pivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing
    return m_drive.run(() -> {});
  }
}