// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// ANGLES FOR AUTO:

//ELEVATOR HEIGHT AT BARS: 0.5, IRL: TBD
//ELEVATOR HEIGHT AT INTAKE STATION: 0.66 (closer to 0.75), IRL: TBD
//PIVOT ANGLE AT INTAKE STATION:56.2 , IRL: -24
// L2 angle 112, elevator 1.55
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double PIVOT_OFFSET = 0;
  public static final double ELEVATOR_OFFSET = 0;

  public static final double ELEVATOR_GEAR_RATIO = (81.0/1.0);
  public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(1.756);
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound, TODO: Replace actual mass
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double ALGAE_IN_SPEED    = 0.2;
  public static final double ALGAE_OUT_SPEED    = 0.4;
  public static final double CLIMBER_SPEED  = 0.5;
  public static final double ELEVATOR_SPEED  = 0.3;
  public static final double INTAKE_SPEED  = 0.4;
  public static final double PIVOT_SPEED  = 0.15;


//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND        = 0.135; //TODO: look into if this is slowing down turning and check physicalproperties.json for if that can increase speed via ramp rate
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

  }

  public static class Vision {
    public static final double xWidth = Units.inchesToMeters(32.5);
    public static final double yWidth = Units.inchesToMeters(29);

    /** 
     * For simulation only.
     * Get properties from https://photonvision.local:5800 
     */
    public static final SimCameraProperties cameraProperties = new SimCameraProperties();
    static {
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(74.10));
        cameraProperties.setCalibError(0.68, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);
    }

    /**
     * Reference coordinate system: 
     * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html 
     */
    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(14.5),
            Units.inchesToMeters(0),
            Units.inchesToMeters(13.5)
        ),
        new Rotation3d(0, 0, 0)
    );
  }
}
