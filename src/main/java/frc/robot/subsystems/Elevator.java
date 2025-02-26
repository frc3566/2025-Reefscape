package frc.robot.subsystems;


import frc.robot.Constants;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Elevator extends SubsystemBase {
    public SparkMax motor, follower;
    public double speed = Constants.ELEVATOR_SPEED;

    public Elevator() {
        motor = new SparkMax(10, MotorType.kBrushless);
        follower = new SparkMax(11, MotorType.kBrushless);
        SparkMaxConfig motorConfig = getMotorConfig(false);
        SparkMaxConfig followerConfig = getMotorConfig(true);
        followerConfig.follow(motor, true);
        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up() {
        motor.set(speed);
    }

    public void down() {
        motor.set(-speed);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setVolatge(double volt) {
        motor.setVoltage(volt);
    }

    public double getHeight() {
        return motor.getEncoder().getPosition(); //TODO: change
    }
 
private SparkMaxConfig getMotorConfig(boolean isInverted) {
    // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(20)
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    motorConfig.encoder
        .positionConversionFactor(360/81);
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.0042864, 0, 0, 0.0675); //TODO: monitor
    return motorConfig;
    }
}