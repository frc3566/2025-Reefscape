package frc.robot.subsystems;


import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SysIdPivot extends SubsystemBase {
    private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
    private final MutDistance       m_distance       = Meters.mutable(0);
    private final MutAngle          m_rotations      = Rotations.mutable(0);
    private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);
    private  SparkMax m_motor = new SparkMax(14, MotorType.kBrushless); // Use one motor as the main one
    private  RelativeEncoder m_encoder =  m_motor.getEncoder();
    private  ProfiledPIDController m_controller = new ProfiledPIDController
          ( 0, //All placeholders for now
            0,
            0,
            new Constraints(0,0)); //TODO: constants
        
    public SysIdPivot() {
        /* Setting up the motors and PID*/
        SparkMaxConfig primaryConfig = getMotorConfig(false);

        m_motor.configure(primaryConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setVoltage(double volt) {
        m_motor.setVoltage(volt);
    }

    private SysIdRoutine      m_sysIdRoutine   =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage. 
            new SysIdRoutine.Config(Volts.per(Second).of(1),
                                    Volts.of(7),
                                    Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                m_motor::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("pivot")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getHeightMeters(),
                                                            Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                    .linearVelocity(m_velocity.mut_replace(getVelocityMetersPerSecond(),
                                                            MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
                },
                this));

    private SparkMaxConfig getMotorConfig(boolean isInverted) {
        // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .smartCurrentLimit(20)
            .inverted(isInverted)
            .idleMode(IdleMode.kBrake);
        return motorConfig;
        }
    /* Calculations */
    public double getHeightMeters(){ //TODO: constants
        // return (m_encoder.getPosition() / ElevatorConstants.kElevatorGearing) *
        // (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
        return 0.0;
    }

    public double getVelocityMetersPerSecond(){ //TODO: constants
    //   return ((.getVelocity() / 60)/ ElevatorConstants.kElevatorGearing) *
    //          (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
        return 0.0;
    }

    public Distance getLinearPosition(){
        return convertRotationsToDistance(Rotations.of(m_encoder.getPosition()));
    }

    public static Distance convertRotationsToDistance(Angle rotations) { //TODO: constants
    //   return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
    //                    (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI));
    // }
    return Meters.of(0);
    }
}