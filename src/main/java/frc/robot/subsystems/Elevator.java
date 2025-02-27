package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Elevator extends SubsystemBase {
    private  SparkMax masterMotor, followerMotor; // Use one motor as the main one
    private  RelativeEncoder encoder;
    private  ProfiledPIDController m_controller;
    private ElevatorFeedforward feedforward;
    private final double speed = 0.2;
    private double ElevatorGearing = 81.0/1.0;
    private double kElevatorDrumRadius = 1.756;

    public Elevator() {
        masterMotor = new SparkMax(10, MotorType.kBrushless); // Use one motor as the main one
        followerMotor = new SparkMax(11, MotorType.kBrushless);
        encoder = masterMotor.getEncoder();



        SparkMaxConfig primaryConfig = getMotorConfig(false);
        SparkMaxConfig followerConfig = getMotorConfig(true);
        followerConfig.follow(masterMotor, true);
        masterMotor.configure(primaryConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    public void up() {
        masterMotor.set(speed);
    }

    public void down() {
        masterMotor.set(-speed);
    }

    public void stop() {
        masterMotor.stopMotor();
    }

    public void setVoltage(double volt) {
        masterMotor.setVoltage(volt);
    }

    public double getRotation() {
        return encoder.getPosition();
    }

    public double getVelocityMetersPerSecond()
    {
      return ((encoder.getVelocity() / 60)/ (ElevatorGearing)) *
             (2 * Math.PI * kElevatorDrumRadius);
    }

private SparkMaxConfig getMotorConfig(boolean isInverted) {
    // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(40)
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    return motorConfig;
    }
}